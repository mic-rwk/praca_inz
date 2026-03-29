#!/usr/bin/env python3
import os
import glob
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import tensorflow as tf
from keras import layers, models, losses, regularizers, optimizers
from keras.callbacks import EarlyStopping, ReduceLROnPlateau
from sklearn.utils import shuffle, resample, class_weight
from sklearn.model_selection import train_test_split
from sklearn.utils.class_weight import compute_sample_weight
import seaborn as sns
from scipy.stats import norm
import random
import gc
import time

TRAIN_FOLDERS = [
    "csv_output/maze_1",
    "csv_output/maze_2",
    "csv_output/willowgarage"
]

TEST_FOLDERS = [
    "processed/maze_3",
]
TEST_FOLDERS_TRAJ = [
    "processed/maze_3_trajectory",
]

MODEL_TYPE = "cnn"   # "cnn" or "gru" "lstm"
MODEL_SAVE_PATH = "./model/cnn_test.keras"
EPOCHS = 50
BATCH_SIZE = 32
SEQUENCE_LENGTH = 2
TARGET_COLS = ["velocity_linear", "velocity_angular"]
TIMESTAMP_CANDIDATE_COLS = ["timestamp", "Timestamp", "time", "Time"]
GRU_VARIANT = "B1"   # "A1", "A2", "B1", "B2"
CNN_VARIANT = "lidar_wheels"  # "lidar" or "lidar_wheels"

BALANCE_LINEAR_VELOCITY = False
LINEAR_BIN_WIDTH = 0.1
LINEAR_MAX_SAMPLES_PER_BIN = 6000

USE_DIFFS = True

ERASE_LIDAR_SAMPLES = True
LIDAR_SAMPLES_TO_LEAVE = 20 #18 samples in result
LIDAR_NOISE = True
WHEELS_NOISE = True

USE_SEQUENCE = False

LIDAR_ZERO_ENABLED = False
LIDAR_ZERO_EVERY_N = 2
LIDAR_ZERO_LEN = 15
LIDAR_ZERO_SEED = 42

LINEAR_VELOCITY_SCALE = 1.2
LINEAR_VELOCITY_OFFSET = 0.6

_LAST_LINEAR_OFFSET = None
_LAST_LINEAR_HALF_RANGE = None

ANGULAR_VELOCITY_SCALE = 3.0

GAMMA = 0.6

def scale_to_gamma_range(y, vmin, vmax, gamma=GAMMA):
    """Skaluje y z zakresu [vmin, vmax] do zakresu [-gamma, gamma]."""
    if vmax <= vmin:
        return np.zeros_like(y)
    
    # Skalowanie do [0, 1]
    y_unit = (y - vmin) / (vmax - vmin)
    
    # Skalowanie do [-gamma, gamma]
    y_scaled = (2 * gamma * y_unit) - gamma
    return y_scaled

def scale_velocities_for_tanh(y):

    global _LAST_LINEAR_OFFSET, _LAST_LINEAR_HALF_RANGE

    y = np.asarray(y)
    y_scaled = y.copy().astype(float)

    linear = y[:, 0]
    try:
        offset = float(np.nanmedian(linear))
    except Exception:
        offset = float(LINEAR_VELOCITY_OFFSET)

    try:
        half_range = float(np.nanmax(np.abs(linear - offset)))
    except Exception:
        half_range = float(LINEAR_VELOCITY_OFFSET)

    if not np.isfinite(half_range) or half_range <= 1e-6:
        half_range = float(LINEAR_VELOCITY_OFFSET)

    y_scaled[:, 0] = (linear - offset) / half_range

    _LAST_LINEAR_OFFSET = offset
    _LAST_LINEAR_HALF_RANGE = half_range

    y_scaled[:, 1] = y[:, 1] / ANGULAR_VELOCITY_SCALE

    return y_scaled

def make_cnn_lidar_only(X_df, lidar_cols):
    L = X_df[lidar_cols].to_numpy()
    return pd.DataFrame(L, columns=lidar_cols)

def make_cnn_lidar_wheels(X_df, lidar_cols, wheel_cols):
    L = X_df[lidar_cols].to_numpy()
    df = pd.DataFrame(L)
    print(df.describe())
    W = X_df[wheel_cols].to_numpy() if len(wheel_cols) > 0 else np.zeros((L.shape[0], 0))
    df = pd.DataFrame(W)
    print(df.describe())
    out = np.hstack([L, W])
    cols = lidar_cols + wheel_cols
    return pd.DataFrame(out, columns=cols)


def plot_feature_velocity_correlations(X_df, y, feature_cols, group="lidar"):
    if group == "lidar":
        mask = [c.startswith("lidar_") for c in feature_cols]
    elif group == "encoder":
        mask = [("encoder" in c.lower()) for c in feature_cols]
    else:
        mask = [True] * len(feature_cols)

    X_sub = X_df.loc[:, np.array(feature_cols)[mask]].to_numpy()
    lin = y[:, 0]
    ang = y[:, 1]

    corr_lin = []
    corr_ang = []

    for i in range(X_sub.shape[1]):
        xi = X_sub[:, i]
        corr_lin.append(np.corrcoef(xi, lin)[0, 1])
        corr_ang.append(np.corrcoef(xi, ang)[0, 1])

    corr_lin = np.array(corr_lin)
    corr_ang = np.array(corr_ang)

def get_csv_files_from_paths(paths):
    if isinstance(paths, str):
        paths = [paths]
    files = []
    for p in paths:
        if os.path.isfile(p) and p.endswith(".csv"):
            files.append(os.path.abspath(p))
        elif os.path.isdir(p):
            files.extend(sorted(glob.glob(os.path.join(p, "*.csv"))))
    return files

def read_and_clean_csv(fp: str) -> pd.DataFrame:
    try:
        df = pd.read_csv(fp, header=0, low_memory=False)
    except:
        print("Brak kolumn\n")
        return

    df.replace(["nil", "None", "-", "", " "], np.nan, inplace=True)
    df = df.infer_objects(copy=False)

    df.replace(["inf", "-inf"], [np.inf, -np.inf], inplace=True)
    df = df.apply(pd.to_numeric, errors="coerce")
    
    df.replace([np.inf, -np.inf], 0.0, inplace=True)

    # df.interpolate(axis=0, method='linear', limit_direction='both', inplace=True)
    # df.fillna(df.median(numeric_only=True), inplace=True)
    
    df.fillna(0, inplace=True)
    
    df.dropna(how="all", inplace=True)
    
    velocity_col = 'linear_velocity' 
    if velocity_col in df.columns:
        df[velocity_col] = df[velocity_col].clip(lower=0.0)
        
        df = df[df[velocity_col] <= 1.2]

    return df

def load_and_process_files(file_list, lidar_cols, target_cols, wheel_cols=None,
                           use_diffs=True, normalize=True):

    X_all = []
    y_all = []

    for fpath in file_list:
        print(f"Process dla: {fpath}")
        df = read_and_clean_csv(fpath)
        if df is None:
            continue
        

        cols = list(lidar_cols)
        if wheel_cols:
            cols += list(wheel_cols)

        df = df[ cols + target_cols ]   

        X = df[cols]
        y = df[target_cols].to_numpy(dtype=np.float32)

        if normalize:
            X_norm, medians, means, stds = impute_and_normalize_train(X)
        else:
            X_norm = X.to_numpy(dtype=np.float32)

        if use_diffs:
            X_norm = make_lidar_diffs(X_norm)
            y = y[1:]

        if X_norm.shape[0] == 0:
            print(f"Empty after diff: {fpath}")
            continue

        X_all.append(X_norm)
        y_all.append(y)

    if not X_all:
        print("No valid data after processing!")
        return np.zeros((0, len(lidar_cols))), np.zeros((0, len(target_cols)))

    return np.vstack(X_all), np.vstack(y_all)







def load_and_concat(csv_files):
    dfs = []
    for f in csv_files:
        try:
            df = read_and_clean_csv(f)
            dfs.append(df)
        except Exception as e:
            print(f"[error] {f}: {e}")
    if not dfs:
        return pd.DataFrame()
    return pd.concat(dfs, ignore_index=True)

def split_features_targets(df):
    cols = df.columns.tolist()

    assert cols[0].lower().startswith("time")
    assert cols[1] == "velocity_linear"
    assert cols[2] == "velocity_angular"

    timestamp_col = cols[0]
    target_cols = ["velocity_linear", "velocity_angular"]

    lidar_cols = [c for c in cols if c.startswith("lidar")]
    if ERASE_LIDAR_SAMPLES:
        lidar_cols = [col for i, col in enumerate(lidar_cols) 
                      if i % LIDAR_SAMPLES_TO_LEAVE == 0]

    wheel_cols = []
    if "encoder_left" in cols:
        wheel_cols.append("encoder_left")
    if "encoder_right" in cols:
        wheel_cols.append("encoder_right")

    X_cols = lidar_cols + wheel_cols
    X = df[X_cols]

    y = df[target_cols].to_numpy(dtype=np.float32)

    return X, y, X_cols, lidar_cols, wheel_cols

def impute_and_normalize_train(X_df):
    medians = X_df.median(axis=0, skipna=True)
    X_df = X_df.fillna(medians)

    X_arr = X_df.to_numpy(dtype=np.float32)
    X_arr = np.nan_to_num(X_arr, nan=0.0, posinf=0.0, neginf=0.0)

    means = np.mean(X_arr, axis=0)
    stds  = np.std(X_arr, axis=0)
    stds[stds == 0] = 1.0

    X_norm = (X_arr - means) / stds

    return X_norm, medians.to_dict(), means, stds

def impute_and_normalize_test(X_df, medians, means, stds):
    X_df = X_df.fillna(pd.Series(medians))
    X_arr = X_df.to_numpy(dtype=np.float32)
    X_arr = np.nan_to_num(X_arr, nan=0.0, posinf=0.0, neginf=0.0)

    X_norm = (X_arr - means) / stds
    return X_norm

def get_shifted_data(data):
    shifted = np.roll(data, shift=1, axis=0)
    shifted[0] = data[0]
    return shifted

def make_input_A1_uniform(X_df, lidar_cols, wheel_cols):
    L = X_df[lidar_cols].to_numpy()
    L_prev = get_shifted_data(L)
    
    out = np.hstack([L_prev, L])
    return pd.DataFrame(out, columns=[f"in_{i}" for i in range(out.shape[1])])

def make_input_A2_interleaved(X_df, lidar_cols, wheel_cols):
    L = X_df[lidar_cols].to_numpy()
    L_prev = get_shifted_data(L)
    N = L.shape[0]
    
    out = np.zeros((N, len(lidar_cols) * 2), dtype=np.float32)
    for i in range(len(lidar_cols)):
        out[:, 2*i]     = L_prev[:, i]
        out[:, 2*i + 1] = L[:, i]
        
    return pd.DataFrame(out, columns=[f"in_{i}" for i in range(out.shape[1])])

def make_input_B1_uniform(X_df, lidar_cols, wheel_cols):
    L = X_df[lidar_cols].to_numpy()
    W = X_df[wheel_cols].to_numpy()
    L_prev = get_shifted_data(L)
    W_prev = get_shifted_data(W)
    
    out = np.hstack([L_prev, L, W_prev, W])
    return pd.DataFrame(out, columns=[f"in_{i}" for i in range(out.shape[1])])

def make_input_B2_interleaved(X_df, lidar_cols, wheel_cols):
    L = X_df[lidar_cols].to_numpy()
    W = X_df[wheel_cols].to_numpy()
    L_prev = get_shifted_data(L)
    W_prev = get_shifted_data(W)
    
    out_list = []
    for i in range(len(lidar_cols)):
        out_list.append(L_prev[:, i])
        out_list.append(L[:, i])
    
    for i in range(len(wheel_cols)):
        out_list.append(W_prev[:, i])
        out_list.append(W[:, i])

    out = np.vstack(out_list).T
    return pd.DataFrame(out, columns=[f"in_{i}" for i in range(out.shape[1])])

def reshape_for_model(X_norm, model_type, seq_len=SEQUENCE_LENGTH):
    if model_type == "cnn":
        if X_norm.ndim == 3:
            return X_norm
        else:
            return np.expand_dims(X_norm, axis=-1)
    elif model_type in ("gru", "lstm"):
        if X_norm.ndim == 3:
            return X_norm
        else:
            return np.expand_dims(X_norm, axis=1)
    else:
        raise ValueError("Unknown model_type")
    
def create_sequences(X, y, window):
    X_seq = []
    y_seq = []
    for i in range(window, len(X)):
        X_seq.append(X[i-window:i, :])
        y_seq.append(y[i])
    return np.array(X_seq), np.array(y_seq)

def build_cnn(input_shape):
    #l2, complexity, zapis z treningu do csv i potem wgrać do wykresu
    model = models.Sequential(name="CNN_velocity_estimator")
    model.add(layers.Input(shape=input_shape))
    # model.add(layers.GaussianNoise(0.001))
    # model.add(layers.Dropout(0.6))
    # model.add(layers.Conv1D(32, kernel_size=5, activation="relu"))
    model.add(layers.Conv1D(16, kernel_size=3, activation="relu"))
    model.add(layers.BatchNormalization())
    model.add(layers.MaxPooling1D(pool_size=2))
    
    # model.add(layers.Dropout(0.3))
    model.add(layers.Conv1D(16, kernel_size=3, activation="relu"))
    # model.add(layers.BatchNormalization())
    model.add(layers.MaxPooling1D(pool_size=2))
    # model.add(layers.Dropout(0.3))
    # model.add(layers.Conv1D(16, kernel_size=5, activation="relu"))
    # model.add(layers.MaxPooling1D(pool_size=4))
    # model.add(layers.Conv1D(16, kernel_size=5, activation="relu", padding="same"))
    # model.add(layers.MaxPooling1D(pool_size=4))
    model.add(layers.Flatten())
    model.add(layers.Dense(32, activation="relu"))
    # model.add(layers.Dropout(0.3))
    # model.add(layers.BatchNormalization())
    model.add(layers.Dense(2, activation="tanh"))
    # model = models.Sequential()
    # model.add(layers.Input(shape=input_shape))
    # model.add(layers.Conv1D(64, 5, activation='relu', kernel_regularizer=regularizers.l2(0.000001)))
    # model.add(layers.MaxPooling1D(pool_size=4))
    # # model.add(layers.Dropout(0.3))
    # model.add(layers.Conv1D(32, 5, activation='relu', kernel_regularizer=regularizers.l2(0.00001)))
    # model.add(layers.MaxPooling1D(pool_size=4))
    # # model.add(layers.Dropout(0.3))
    # model.add(layers.Conv1D(16, 5, activation='relu'))
    # model.add(layers.MaxPooling1D(pool_size=4))
    # model.add(layers.Flatten())
    # model.add(layers.Dense(128, activation='relu'))
    # model.add(layers.Dense(2, activation="tanh"))
    optimizer = optimizers.Adam(learning_rate=0.001)
    # losses.Huber(0.4)
    model.compile(optimizer=optimizer, loss="mse", metrics=["mse", "mae"])
    return model

def normalize_inputs_tanh(X_df, lidar_cols, wheel_cols):

    X = X_df.copy()

    stats = {
        "lidar_means": {},
        "lidar_stds": {},
        "wheel_means": {},
        "wheel_stds": {}
    }

    for col in lidar_cols:
        col_vals = X[col].values.astype(np.float32)

        mean = np.mean(col_vals)
        std = np.std(col_vals)
        if std == 0:
            std = 1.0

        stats["lidar_means"][col] = float(mean)
        stats["lidar_stds"][col] = float(std)

        X[col] = 0.5 * np.tanh((col_vals - mean) / std) + 0.5

    for col in wheel_cols:
        col_vals = X[col].values.astype(np.float32)

        mean = np.mean(col_vals)
        std = np.std(col_vals)
        if std == 0:
            std = 1.0

        stats["wheel_means"][col] = float(mean)
        stats["wheel_stds"][col] = float(std)

        X[col] = 0.5 * np.tanh((col_vals - mean) / std) + 0.5

    return X, stats

def normalize_inputs_tanh_test(X_df, lidar_cols, wheel_cols, stats):
    X = X_df.copy()

    for col in lidar_cols:
        mean = stats["lidar_means"][col]
        std = stats["lidar_stds"][col]
        X[col] = 0.5 * np.tanh((X[col] - mean) / std) + 0.5

    for col in wheel_cols:
        mean = stats["wheel_means"][col]
        std = stats["wheel_stds"][col]
        X[col] = 0.5 * np.tanh((X[col] - mean) / std) + 0.5

    return X


def add_noise_lidar(df, lidar_cols, std_fraction=0.05):
    df = df.copy()
    for col in lidar_cols:
        max_val = df[col].max()
        noise = np.random.normal(
            loc=0.0,
            scale=std_fraction * max_val,
            size=len(df)
        )
        df[col] += noise
    return df

def add_noise_wheels(df, wheel_cols, std=0.002):
    df = df.copy()
    for col in wheel_cols:
        max_val = df[col].max()
        noise = np.random.normal(0, std*max_val, len(df))
        df[col] += noise
    return df


def apply_lidar_zero_pattern(df, lidar_cols, every_n=LIDAR_ZERO_EVERY_N, zero_len=LIDAR_ZERO_LEN, seed=LIDAR_ZERO_SEED):

    if not LIDAR_ZERO_ENABLED:
        return df

    if len(lidar_cols) == 0:
        return df

    n_rows = len(df)
    if n_rows == 0:
        return df

    rng = np.random.RandomState(seed)
    for start in range(every_n - 1, n_rows, every_n):
        lidar_idx = rng.randint(0, len(lidar_cols))
        col = lidar_cols[lidar_idx]
        end = min(n_rows, start + zero_len)
        try:
            df.loc[start:end-1, col] = 0.0
        except Exception:
            vals = df[col].to_numpy()
            vals[start:end] = 0.0
            df[col] = vals

    return df

def build_gru(input_shape):
    model = models.Sequential(name="GRU_velocity_estimator")
    model.add(layers.Input(shape=input_shape))
    # 2 layer
    model.add(layers.GRU(40, return_sequences=True))
    model.add(layers.GRU(40, return_sequences=True))
    #3 layer
    # model.add(layers.GRU(40, return_sequences=True))
    # model.add(layers.GRU(40, return_sequences=True))
    # model.add(layers.GRU(40, return_sequences=True))
    # model.add(layers.GRU(40, return_sequences=False))
    model.add(layers.Flatten())
    model.add(layers.Dense(2, activation="tanh"))
    model.compile(optimizer="adam", loss="mse", metrics=["mse", "mae"])
    return model

def build_lstm(input_shape):
    model = models.Sequential(name="LSTM_velocity_estimator")
    model.add(layers.Input(shape=input_shape))
    # 2 layer
    model.add(layers.LSTM(40, return_sequences=True))
    model.add(layers.LSTM(40))
    # 3 layer
    # model.add(layers.LSTM(40, return_sequences=True))
    # model.add(layers.LSTM(40, return_sequences=True))
    # model.add(layers.LSTM(40, return_sequences=True))
    # model.add(layers.LSTM(40, return_sequences=False))
    model.add(layers.Flatten())
    model.add(layers.Dense(2, activation="tanh"))
    model.compile(optimizer="adam", loss="mse", metrics=["mse", "mae"])
    return model

def train_model(X_train, y_train, model_type):
    # Xr = reshape_for_model(X_train, model_type)
    input_shape = X_train.shape[1:]
    
    print(f"Input shape: {input_shape}")
    print(f"X_train stats - min: {X_train.min():.4f}, max: {X_train.max():.4f}, mean: {X_train.mean():.4f}")
    print(f"y_train stats - min: {y_train.min():.4f}, max: {y_train.max():.4f}, mean: {y_train.mean():.4f}")
        
    if model_type == "cnn":
        model = build_cnn(input_shape)
    elif model_type == "gru":
        model = build_gru(input_shape)
    elif model_type == "lstm":
        model = build_lstm(input_shape)

    print("y_train finite:", np.isfinite(y_train).all(), " y_train NaNs:", np.isnan(y_train).sum())
        
    callbacks = []
    if model_type in ["gru", "lstm", "cnn"]:
        callbacks.append(
            EarlyStopping(
                monitor='val_loss',
                patience=10,
                restore_best_weights=True
            )
    )
        
    history = model.fit(
        X_train, y_train, 
        # sample_weight=sample_weights,
        epochs=EPOCHS, 
        batch_size=BATCH_SIZE,
        verbose=2,
        validation_split=0.2,
        shuffle=True,
        callbacks=callbacks
    )
    return model, history


LINEAR_VEL_MIN = 0.0
LINEAR_VEL_MAX = 1.2

# _LAST_LINEAR_VMIN = None
# _LAST_LINEAR_VMAX = None

_LAST_LINEAR_VMIN = 0
_LAST_LINEAR_VMAX = 1.2

ANGULAR_VEL_MIN = -3.0
ANGULAR_VEL_MAX = 3.0

LIDAR_MIN = -9.0 #0.0
LIDAR_MAX = 9.0 #12.0

WHEEL_MIN = -15.0
WHEEL_MAX = 30.0


def scale_to_tanh_range(values, vmin, vmax):
    return 2.0 * (values - vmin) / (vmax - vmin) - 1.0

def unscale_from_tanh_range(values, vmin, vmax):
    return (values + 1.0) * (vmax - vmin) / 2.0 + vmin

def scale_outputs_for_tanh(y):
    global _LAST_LINEAR_VMIN, _LAST_LINEAR_VMAX

    y = np.asarray(y)
    y_scaled = np.zeros_like(y, dtype=np.float32)

    try:
        vmin = float(LINEAR_VEL_MIN)
        vmax = float(LINEAR_VEL_MAX)
    except Exception:
        vmin = np.nan
        vmax = np.nan

    if not np.isfinite(vmin) or not np.isfinite(vmax) or (vmax - vmin) < 1e-6:
        vmin = float(np.nanmin(y[:, 0]))
        vmax = float(np.nanmax(y[:, 0]))
        pad = max(0.05 * (vmax - vmin), 1e-3)
        vmin -= pad
        vmax += pad

    _LAST_LINEAR_VMIN = vmin
    _LAST_LINEAR_VMAX = vmax

    # y_scaled[:, 0] = scale_to_tanh_range(y[:, 0], vmin, vmax)
    # y_scaled[:, 1] = scale_to_tanh_range(y[:, 1], ANGULAR_VEL_MIN, ANGULAR_VEL_MAX)
    y_scaled[:, 0] = scale_to_gamma_range(y[:, 0], vmin, vmax, gamma=GAMMA)
    y_scaled[:, 1] = scale_to_gamma_range(y[:, 1], ANGULAR_VEL_MIN, ANGULAR_VEL_MAX, gamma=GAMMA)
    return y_scaled

def unscale_velocities_from_tanh(y_scaled):
    global _LAST_LINEAR_VMIN, _LAST_LINEAR_VMAX

    y_scaled = np.asarray(y_scaled)
    y = y_scaled.copy().astype(float)

    vmin_L = _LAST_LINEAR_VMIN if _LAST_LINEAR_VMIN is not None else float(LINEAR_VEL_MIN)
    vmax_L = _LAST_LINEAR_VMAX if _LAST_LINEAR_VMAX is not None else float(LINEAR_VEL_MAX)
    
    y_unit_L = (y_scaled[:, 0] + GAMMA) / (2 * GAMMA)
    
    y[:, 0] = y_unit_L * (vmax_L - vmin_L) + vmin_L

    vmin_A = -3.0 if ANGULAR_VEL_MIN is not None else float(ANGULAR_VEL_MIN)
    vmax_A = 3.0 if ANGULAR_VEL_MAX is not None else float(ANGULAR_VEL_MAX)

    y_unit_A = (y_scaled[:, 1] + GAMMA) / (2 * GAMMA)
    
    y[:, 1] = y_unit_A * (vmax_A - vmin_A) + vmin_A
    
    return y

def scale_inputs_for_tanh(X_df, lidar_cols, wheel_cols):

    X_scaled = X_df.copy()
    
    for col in lidar_cols:
        if col in X_scaled.columns:
            X_scaled[col] = scale_to_tanh_range(X_scaled[col], LIDAR_MIN, LIDAR_MAX)
    
    for col in wheel_cols:
        if col in X_scaled.columns:
            X_scaled[col] = np.clip(X_scaled[col], WHEEL_MIN, WHEEL_MAX)
            X_scaled[col] = scale_to_tanh_range(X_scaled[col], WHEEL_MIN, WHEEL_MAX)
    
    return X_scaled

def compute_trajectory_from_velocities(vel, dt=0.1):
    """
    vel: [N, 2] -> [v_linear, v_angular]
    Zwraca tablicę [N, 2] -> [x, y]
    """
    N = len(vel)
    traj = np.zeros((N, 2))
    x, y, theta = 0.0, 0.0, 0.0

    for i in range(1, N):
        v = vel[i, 0]
        w = vel[i, 1]

        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += w * dt

        traj[i] = [x, y]

    return traj


def extract_odometry_velocities_from_df(df, wheel_cols, *, ticks_per_rev=None, wheel_radius=0.05, wheel_base=0.35, flip_sign=None):
    
    if len(wheel_cols) < 2 or not all(col in df.columns for col in wheel_cols[:2]):
        print("Brak danych z enkoderów")
        return None

    wL_raw = df[wheel_cols[0]].to_numpy(dtype=float)
    wR_raw = df[wheel_cols[1]].to_numpy(dtype=float)

    flipL = 1.0
    flipR = 1.0
    if flip_sign is True:
        flipL = -1.0
        flipR = -1.0
    elif isinstance(flip_sign, (list, tuple)) and len(flip_sign) == 2:
        if flip_sign[0]:
            flipL = -1.0
        if flip_sign[1]:
            flipR = -1.0

    wL_raw = flipL * wL_raw
    wR_raw = flipR * wR_raw

    dt = None
    for tcol in ("timestamp", "Timestamp", "time", "Time"):
        if tcol in df.columns:
            ts = pd.to_numeric(df[tcol], errors="coerce")
            if ts.notna().sum() > 1:
                diffs = np.diff(ts.to_numpy(dtype=float))
                median = float(np.median(diffs))
                if median > 1e6:
                    dt = median / 1e9
                elif median > 1e3:
                    dt = median / 1e3
                elif median >= 1e-3:
                    dt = median
                else:
                    dt = max(median, 1e-3)
            break
    if dt is None:
        dt = 0.1

    as_ticks = False
    if ticks_per_rev is not None:
        as_ticks = True
    else:
        name_hint = ("encoder" in wheel_cols[0].lower()) or ("encoder" in wheel_cols[1].lower())
        def is_integer_like(arr):
            arr = np.asarray(arr)
            ok = np.isfinite(arr)
            if ok.sum() == 0:
                return False
            frac = np.abs(arr[ok] - np.round(arr[ok])) < 1e-6
            return (frac.mean() > 0.9)

        mag_threshold = 1.0
        L_inty = is_integer_like(wL_raw)
        R_inty = is_integer_like(wR_raw)
        L_mag = np.nanmedian(np.abs(wL_raw)) if np.isfinite(np.nanmedian(wL_raw)) else 0.0
        R_mag = np.nanmedian(np.abs(wR_raw)) if np.isfinite(np.nanmedian(wR_raw)) else 0.0
        mag_ok = (L_mag > mag_threshold) or (R_mag > mag_threshold)

        if name_hint and (L_inty or R_inty) and mag_ok:
            as_ticks = True
        else:
            as_ticks = False


    if as_ticks:
        if ticks_per_rev is None:
            mean_enc = (wL_raw + wR_raw) / 2.0
            diff_enc = (wR_raw - wL_raw) / 2.0

            s_lin = 0.0
            s_ang = 0.0
            intercept_lin = 0.0
            intercept_ang = 0.0
            min_samples = 8
            if 'velocity_linear' in df.columns:
                y_lin = pd.to_numeric(df['velocity_linear'], errors='coerce').to_numpy(dtype=float)
                mask = np.isfinite(y_lin) & np.isfinite(mean_enc)
                if mask.sum() >= min_samples and np.abs(mean_enc[mask]).sum() > 0:
                    A = np.vstack([mean_enc[mask], np.ones(mask.sum())]).T
                    sol, _, _, _ = np.linalg.lstsq(A, y_lin[mask], rcond=None)
                    s_lin, intercept_lin = float(sol[0]), float(sol[1])
            if 'velocity_angular' in df.columns:
                y_ang = pd.to_numeric(df['velocity_angular'], errors='coerce').to_numpy(dtype=float)
                mask2 = np.isfinite(y_ang) & np.isfinite(diff_enc)
                if mask2.sum() >= min_samples and np.abs(diff_enc[mask2]).sum() > 0:
                    B = np.vstack([diff_enc[mask2], np.ones(mask2.sum())]).T
                    sol2, _, _, _ = np.linalg.lstsq(B, y_ang[mask2], rcond=None)
                    s_ang, intercept_ang = float(sol2[0]), float(sol2[1])

            trusted = False
            best_choice = None
            try:
                from scipy.stats import pearsonr
                best_score = -1.0
                best_params = None
                for flipL in (1.0, -1.0):
                    for flipR in (1.0, -1.0):
                        m_enc = (flipL * wL_raw + flipR * wR_raw) / 2.0
                        d_enc = (flipR * wR_raw - flipL * wL_raw) / 2.0
                        score = 0.0
                        if 'velocity_linear' in df.columns:
                            y_lin = pd.to_numeric(df['velocity_linear'], errors='coerce').to_numpy(dtype=float)
                            mask = np.isfinite(y_lin) & np.isfinite(m_enc)
                            if mask.sum() >= min_samples and np.abs(m_enc[mask]).sum() > 0:
                                corr_lin = abs(pearsonr(m_enc[mask], y_lin[mask])[0])
                                score = max(score, corr_lin)
                        if 'velocity_angular' in df.columns:
                            y_ang = pd.to_numeric(df['velocity_angular'], errors='coerce').to_numpy(dtype=float)
                            mask2 = np.isfinite(y_ang) & np.isfinite(d_enc)
                            if mask2.sum() >= min_samples and np.abs(d_enc[mask2]).sum() > 0:
                                corr_ang = abs(pearsonr(d_enc[mask2], y_ang[mask2])[0])
                                score = max(score, corr_ang)
                        if score > best_score:
                            best_score = score
                            best_params = (flipL, flipR, score)
                if best_params is not None and best_params[2] > 0.2:
                    flipL, flipR, _ = best_params
                    mean_enc = (flipL * wL_raw + flipR * wR_raw) / 2.0
                    diff_enc = (flipR * wR_raw - flipL * wL_raw) / 2.0
                    if 'velocity_linear' in df.columns:
                        y_lin = pd.to_numeric(df['velocity_linear'], errors='coerce').to_numpy(dtype=float)
                        mask = np.isfinite(y_lin) & np.isfinite(mean_enc)
                        if mask.sum() >= min_samples:
                            A = np.vstack([mean_enc[mask], np.ones(mask.sum())]).T
                            sol, _, _, _ = np.linalg.lstsq(A, y_lin[mask], rcond=None)
                            s_lin, intercept_lin = float(sol[0]), float(sol[1])
                    if 'velocity_angular' in df.columns:
                        y_ang = pd.to_numeric(df['velocity_angular'], errors='coerce').to_numpy(dtype=float)
                        mask2 = np.isfinite(y_ang) & np.isfinite(diff_enc)
                        if mask2.sum() >= min_samples:
                            B = np.vstack([diff_enc[mask2], np.ones(mask2.sum())]).T
                            sol2, _, _, _ = np.linalg.lstsq(B, y_ang[mask2], rcond=None)
                            s_ang, intercept_ang = float(sol2[0]), float(sol2[1])

                    trusted = False
                    if 'velocity_linear' in df.columns and abs(s_lin) > 1e-6:
                        y_lin = pd.to_numeric(df['velocity_linear'], errors='coerce').to_numpy(dtype=float)
                        mask = np.isfinite(y_lin) & np.isfinite(mean_enc)
                        if mask.sum() >= min_samples:
                            corr_lin = pearsonr(mean_enc[mask], y_lin[mask])[0]
                            trusted = trusted or (abs(corr_lin) > 0.25)
                    if 'velocity_angular' in df.columns and abs(s_ang) > 1e-6:
                        y_ang = pd.to_numeric(df['velocity_angular'], errors='coerce').to_numpy(dtype=float)
                        mask2 = np.isfinite(y_ang) & np.isfinite(diff_enc)
                        if mask2.sum() >= min_samples:
                            corr_ang = pearsonr(diff_enc[mask2], y_ang[mask2])[0]
                            trusted = trusted or (abs(corr_ang) > 0.25)
                    best_choice = best_params
                else:
                    trusted = (abs(s_lin) > 1e-6) or (abs(s_ang) > 1e-6)
            except Exception:
                trusted = (abs(s_lin) > 1e-6) or (abs(s_ang) > 1e-6)

            if not trusted:
                print("Automatic calibration not trusted — treating wheel columns as angular velocities (rad/s)")
                omega_L = wL_raw
                omega_R = wR_raw
            else:
                sign_note = ''
                try:
                    if 'velocity_linear' in df.columns and abs(s_lin) > 1e-6:
                        y_lin = pd.to_numeric(df['velocity_linear'], errors='coerce').to_numpy(dtype=float)
                        mask = np.isfinite(y_lin) & np.isfinite(mean_enc)
                        corr_lin = pearsonr(mean_enc[mask], y_lin[mask])[0]
                        if corr_lin < 0:
                            s_lin = -s_lin
                            sign_note = ' (linear sign flipped due to negative correlation)'
                except Exception:
                    pass

                print(f"Estimated affine scales: s_lin={s_lin:.6g}, intercept_lin={intercept_lin:.6g}, s_ang={s_ang:.6g}, intercept_ang={intercept_ang:.6g}{sign_note}")
                v_linear = mean_enc * s_lin + intercept_lin
                v_angular = diff_enc * s_ang + intercept_ang
                return np.column_stack([v_linear, v_angular])
            try:
                if ticks_per_rev is None:
                    d_enc = (np.diff(df[wheel_cols[0]].to_numpy(dtype=float), prepend=0) + np.diff(df[wheel_cols[1]].to_numpy(dtype=float), prepend=0)) / 2.0
                    if dt is not None:
                        ticks_per_sec = d_enc / dt
                        v_lin_arr = pd.to_numeric(df.get('velocity_linear', pd.Series([np.nan]*len(d_enc))), errors='coerce').to_numpy(dtype=float)
                        mask = np.isfinite(ticks_per_sec) & np.isfinite(v_lin_arr) & (np.abs(v_lin_arr) > 1e-3)
                        if mask.sum() > 5:
                            tpr_samples = ticks_per_sec[mask] * (2.0 * np.pi * wheel_radius) / np.abs(v_lin_arr[mask])
                            tpr_samples = tpr_samples[np.isfinite(tpr_samples) & (tpr_samples > 0) & (tpr_samples < 1e6)]
                            if len(tpr_samples) > 0:
                                tpr_med = float(np.median(tpr_samples))
                                if 10 <= tpr_med <= 100000:
                                    ticks_per_rev = tpr_med
                                    print(f"Estimated ticks_per_rev ~= {ticks_per_rev:.1f} (accepted)")
                                else:
                                    pass
            except Exception:
                pass
        else:
            ticks_to_rad = 2.0 * np.pi / float(ticks_per_rev)
            omega_L = (wL_raw / dt) * ticks_to_rad
            omega_R = (wR_raw / dt) * ticks_to_rad
    else:
        omega_L = wL_raw
        omega_R = wR_raw

    v_linear_raw = (omega_L + omega_R) * wheel_radius / 2.0
    v_angular = (omega_R - omega_L) * wheel_radius / wheel_base

    if flip_sign is True:
        v_linear = -v_linear_raw
    elif flip_sign is False:
        v_linear = v_linear_raw
    else:
        v_linear = v_linear_raw

    return np.column_stack([v_linear, v_angular])

def plot_three_trajectories(y_real, y_odom, y_pred, dt=0.1, save_path=None, odom_transform=None):
    """
    Rysuje 2 oddzielne wykresy: (1) trajektorie 2D, (2) błąd pozycji w czasie.
    """
    N = min(len(y_real), len(y_pred), len(y_odom) if y_odom is not None else len(y_pred))
    y_real = y_real[:N]
    y_pred = y_pred[:N]
    if y_odom is not None:
        y_odom = y_odom[:N]

    traj_real = compute_trajectory_from_velocities(y_real, dt)
    traj_pred = compute_trajectory_from_velocities(y_pred, dt)
    
    if y_odom is not None:
        traj_odom = compute_trajectory_from_velocities(y_odom, dt)
        if odom_transform is not None:
            try:
                if odom_transform.get('swap_axes', False):
                    traj_odom = traj_odom[:, [1, 0]]
                if odom_transform.get('flip_x', False):
                    traj_odom[:, 0] = -traj_odom[:, 0]
                if odom_transform.get('flip_y', False):
                    traj_odom[:, 1] = -traj_odom[:, 1]
            except Exception:
                pass

        error_odom = np.linalg.norm(traj_real - traj_odom, axis=1)
        final_error_odom = error_odom[-1]
    else:
        traj_odom = None
        error_odom = None
        final_error_odom = None
    
    error_pred = np.linalg.norm(traj_real - traj_pred, axis=1)
    final_error_pred = error_pred[-1]
    
    print(f"\n{'='*70}")
    print(f"PORÓWNANIE TRAJEKTORII")
    print(f"{'='*70}")
    print(f"Długość trajektorii: {N} próbek ({N*dt:.1f}s)")
    if final_error_odom is not None:
        print(f"Błąd końcowy odometrii: {final_error_odom:.3f} m")
    print(f"Błąd końcowy sieci neuronowej: {final_error_pred:.3f} m")
    print(f"{'='*70}\n")
    
    plt.figure(figsize=(10, 8))
    plt.plot(traj_real[:, 0], traj_real[:, 1], 'g-', linewidth=3, label='Rzeczywista', alpha=0.9)
    
    if traj_odom is not None:
        plt.plot(traj_odom[:, 0], traj_odom[:, 1], 'b--', linewidth=2, label='Odometria', alpha=0.7)
    
    plt.plot(traj_pred[:, 0], traj_pred[:, 1], 'r-.', linewidth=2, label='NN estymacja', alpha=0.7)
    
    plt.plot(traj_real[0, 0], traj_real[0, 1], 'go', markersize=15, markeredgecolor='black', markeredgewidth=2, label='Start')
    plt.plot(traj_real[-1, 0], traj_real[-1, 1], 'gs', markersize=14, markeredgecolor='black', markeredgewidth=2, label='Koniec')
    
    if traj_odom is not None:
        plt.plot(traj_odom[-1, 0], traj_odom[-1, 1], 'bs', markersize=12, markeredgecolor='black', markeredgewidth=1.5)
    
    plt.plot(traj_pred[-1, 0], traj_pred[-1, 1], 'rs', markersize=12, markeredgecolor='red', markerfacecolor='none', markeredgewidth=2.5)
    
    plt.xlabel('X [m]', fontsize=13, fontweight='bold')
    plt.ylabel('Y [m]', fontsize=13, fontweight='bold')
    plt.legend(fontsize=11, loc='best')
    plt.grid(True, alpha=0.3, linestyle='--')
    plt.axis('equal')
    plt.tight_layout()
    if save_path:
        plt.savefig(f"traj_2d_{save_path}", dpi=150, bbox_inches='tight')
    plt.show()

    plt.figure(figsize=(10, 6))
    time = np.arange(N) * dt
    
    if error_odom is not None:
        plt.plot(time, error_odom, 'b-', linewidth=2.5, label=f'Błąd odometrii (ostatecznie: {final_error_odom:.3f}m)')
    
    plt.plot(time, error_pred, 'r-', linewidth=2.5, label=f'Błąd estymacji (ostatecznie: {final_error_pred:.3f}m)')
    
    plt.xlabel('Czas [s]', fontsize=13, fontweight='bold')
    plt.ylabel('Błąd pozycji [m]', fontsize=13, fontweight='bold')
    plt.legend(fontsize=11, loc='best')
    plt.grid(True, alpha=0.3, linestyle='--')
    plt.tight_layout()
    if save_path:
        plt.savefig(f"error_time_{save_path}", dpi=150, bbox_inches='tight')
    plt.show()

# def plot_three_trajectories(y_real, y_odom, y_pred, dt=0.1, save_path=None, odom_transform=None):
#     """
#     Rysuje 3 trajektorie na jednym wykresie: rzeczywistą, odometryczną i estymowaną.
    
#     Parametry:
#     - y_real: array [N, 2] - rzeczywiste prędkości (ground truth)
#     - y_odom: array [N, 2] - prędkości z odometrii (enkodery), może być None
#     - y_pred: array [N, 2] - prędkości predykowane przez sieć
#     - dt: krok czasowy
#     - save_path: ścieżka do zapisu wykresu (opcjonalnie)
#     """

#     # --- 1. Upewnijmy się że długości są równe ---
#     N = min(len(y_real), len(y_pred), len(y_odom) if y_odom is not None else len(y_pred))
#     y_real = y_real[:N]
#     y_pred = y_pred[:N]
#     if y_odom is not None:
#         y_odom = y_odom[:N]

#     # --- 2. Oblicz trajektorie ---
#     traj_real = compute_trajectory_from_velocities(y_real, dt)
#     traj_pred = compute_trajectory_from_velocities(y_pred, dt)

#     print("Y_traj_real:")
#     print(traj_real[:10])

#     print("Y_trtraj_pred")
#     print(traj_pred[:10])
    
#     if y_odom is not None:
#         traj_odom = compute_trajectory_from_velocities(y_odom, dt)
#         # Apply optional simple odometry transform (swap axes / flip signs) if provided
#         if odom_transform is not None:
#             try:
#                 if odom_transform.get('swap_axes', False):
#                     traj_odom = traj_odom[:, [1, 0]]
#                 if odom_transform.get('flip_x', False):
#                     traj_odom[:, 0] = -traj_odom[:, 0]
#                 if odom_transform.get('flip_y', False):
#                     traj_odom[:, 1] = -traj_odom[:, 1]
#             except Exception:
#                 pass

#         error_odom = np.linalg.norm(traj_real - traj_odom, axis=1)
#         final_error_odom = error_odom[-1]
#         print("traj_odom")
#         print(traj_odom[:10])
#         print(f"Final GT pose: x={traj_real[-1,0]:.3f}, y={traj_real[-1,1]:.3f}")
#         print(f"Final Odom pose: x={traj_odom[-1,0]:.3f}, y={traj_odom[-1,1]:.3f}")
#         print(f"Max Ground Truth X, Y: {np.max(np.abs(traj_real[:, 0])):.3f} m, {np.max(np.abs(traj_real[:, 1])):.3f} m")
#         print(f"Max Odom X, Y:         {np.max(np.abs(traj_odom[:, 0])):.3f} m, {np.max(np.abs(traj_odom[:, 1])):.3f} m")
#     else:
#         print("Brak odom")
#         traj_odom = None
#         error_odom = None
#         final_error_odom = None
    
#     error_pred = np.linalg.norm(traj_real - traj_pred, axis=1)
#     final_error_pred = error_pred[-1]
    
#     # --- 3. INFO ---
#     print(f"\n{'='*70}")
#     print(f"PORÓWNANIE TRAJEKTORII")
#     print(f"{'='*70}")
#     print(f"Długość trajektorii: {N} próbek ({N*dt:.1f}s)")
#     if final_error_odom is not None:
#         print(f"Błąd końcowy odometrii: {final_error_odom:.3f} m")
#     print(f"Błąd końcowy sieci neuronowej: {final_error_pred:.3f} m")
#     if final_error_odom is not None:
#         print(f"Poprawa: {(final_error_odom - final_error_pred):.3f} m "
#               f"({100*(1 - final_error_pred/final_error_odom):.1f}%)")
#     print(f"{'='*70}\n")
    
#     # --- 4. Wykresy ---
#     fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    
#     # === 4A. Trajektorie 2D ===
#     axes[0].plot(traj_real[:, 0], traj_real[:, 1], 'g-', linewidth=3, label='Rzeczywista', alpha=0.9)
    
#     if traj_odom is not None:
#         axes[0].plot(traj_odom[:, 0], traj_odom[:, 1], 'b--', linewidth=2,
#                      label='Odometria', alpha=0.7)
    
#     axes[0].plot(traj_pred[:, 0], traj_pred[:, 1], 'r-.', linewidth=2,
#                  label='NN estymacja', alpha=0.7)
    
#     # Start & koniec
#     axes[0].plot(traj_real[0, 0], traj_real[0, 1], 'go', markersize=15,
#                  markeredgecolor='black', markeredgewidth=2, label='Start')
#     axes[0].plot(traj_real[-1, 0], traj_real[-1, 1], 'gs', markersize=14,
#                  markeredgecolor='black', markeredgewidth=2, label='Koniec')
    
#     if traj_odom is not None:
#         axes[0].plot(traj_odom[-1, 0], traj_odom[-1, 1], 'bs', markersize=12,
#                      markeredgecolor='black', markeredgewidth=1.5)
    
#     axes[0].plot(traj_pred[-1, 0], traj_pred[-1, 1], 'rs', markersize=12,
#                  markeredgecolor='red', markerfacecolor='none', markeredgewidth=2.5)
    
#     axes[0].set_xlabel('X [m]', fontsize=13, fontweight='bold')
#     axes[0].set_ylabel('Y [m]', fontsize=13, fontweight='bold')
#     axes[0].legend(fontsize=11, loc='best')
#     axes[0].grid(True, alpha=0.3, linestyle='--')
#     axes[0].axis('equal')
    
#     # === 4B. Błąd w czasie ===
#     time = np.arange(N) * dt
    
#     if error_odom is not None:
#         axes[1].plot(time, error_odom, 'b-', linewidth=2.5,
#                      label=f'Błąd odometrii (ostatecznie: {final_error_odom:.3f}m)')
#     axes[1].plot(time, error_pred, 'r-', linewidth=2.5,
#                  label=f'Błąd estymacji (ostatecznie: {final_error_pred:.3f}m)')
    
#     axes[1].set_xlabel('Czas [s]', fontsize=13, fontweight='bold')
#     axes[1].set_ylabel('Błąd pozycji [m]', fontsize=13, fontweight='bold')
#     axes[1].legend(fontsize=11, loc='best')
#     axes[1].grid(True, alpha=0.3, linestyle='--')
    
#     plt.tight_layout()

#     # Zapis
#     if save_path:
#         plt.savefig(save_path, dpi=150, bbox_inches='tight')
#         print(f"Wykres trajektorii zapisany: {save_path}")
    
#     plt.show()

def load_single_test_csv(folder_path):
    files = [f for f in os.listdir(folder_path) if f.endswith(".csv")]
    if len(files) != 1:
        raise ValueError(f"Folder {folder_path} powinien zawierać 1 plik CSV, a znaleziono {len(files)}")
    path = os.path.join(folder_path, files[0])
    df = pd.read_csv(path)
    try:
        df._source_file = path
    except Exception:
        try:
            df.attrs['_source_file'] = path
        except Exception:
            pass
    return df

def make_lidar_diffs(X_df):
    """
    X_df: DataFrame lub np.array shape (N, F)
    zwraca: numpy array shape (N-1, F) = X[i+1] - X[i]
    """
    X_arr = X_df.to_numpy(dtype=np.float32) if isinstance(X_df, pd.DataFrame) else np.asarray(X_df, dtype=np.float32)
    # jeśli pusta lub tylko 1 wiersz -> zwróć puste
    if X_arr.shape[0] < 2:
        return np.empty((0, X_arr.shape[1]), dtype=np.float32)
    diffs = X_arr[1:, :] - X_arr[:-1, :]
    return diffs

def main(train_folders, test_folders, model_type, model_save_path):
    gc.set_threshold(0)
    train_files = get_csv_files_from_paths(train_folders)
    test_files = get_csv_files_from_paths(test_folders)
    print(f"Found {len(train_files)} train files, {len(test_files)} test files.")
    random_state=59

    df_tmp = pd.read_csv(train_files[0])
    X_tmp, _, features_cols, lidar_cols, wheel_cols = split_features_targets(df_tmp)
    del df_tmp

    X_train_df, y_train = load_and_process_files(
            train_files,
            lidar_cols=lidar_cols,
            target_cols=["velocity_linear", "velocity_angular"],
            use_diffs=USE_DIFFS,
            normalize=True,
            wheel_cols=wheel_cols
        )

    # X_train_df, y_train = load_and_process_files(
    #     train_files,
    #     lidar_cols=lidar_cols,
    #     target_cols=["velocity_linear", "velocity_angular"],       # dostosuj jeśli nazwy inne
    #     use_diffs=USE_DIFFS,
    #     normalize=True,                # per-file normalization → super stabilne LSTM/CNN
    #     wheel_cols=wheel_cols
    # )
    if len(test_files) > 0:
        X_test_df, y_test = load_and_process_files(
            test_files,
            lidar_cols=lidar_cols,
            target_cols=["velocity_linear", "velocity_angular"],
            use_diffs=USE_DIFFS,
            normalize=True,
            wheel_cols=wheel_cols
        )
    else:
        X_test_df, y_test = None, None

    X_train_df = pd.DataFrame(X_train_df, columns=lidar_cols + wheel_cols)
    X_test_df = pd.DataFrame(X_test_df, columns=lidar_cols + wheel_cols)

    print("RAW shapes:", X_train_df.shape, y_train.shape)
    if X_test_df is not None:
        print("RAW test shapes:", X_test_df.shape, y_test.shape)

    # X_train_df, stats = minmax_scale_inputs_train(X_train_df, lidar_cols, wheel_cols)

    # X_test_df = minmax_scale_inputs_test(X_test_df, lidar_cols, wheel_cols, stats)
    # uniform = {
    #     # Equal count for all bins
    #     -1 : 10000, 0: 10000, 1: 20000, 2: 20000, 3: 20000, 4: 20000,
    #     5: 20000, 6: 20000, 7: 20000, 8: 20000, 9: 20000,
    #     10: 20000, 11: 20000
    # }
    #CNN z rysunku + GRU + LSTM
    # uniform = {
    #     # Equal count for all bins
    #     -1 : 10000, 0: 10000, 1: 20000, 2: 40000, 3: 30000, 4: 30000,
    #     5: 30000, 6: 30000, 7: 20000, 8: 20000, 9: 30000,
    #     10: 30000, 11: 30000
    # }
    uniform = {
    -1 : 20000, 
    0: 30000, 1: 30000, 2: 30000, 3: 30000,
    4: 30000, 5: 35000, 6: 35000, 7: 35000,
    8: 40000, 9: 45000, 10: 50000, 11: 50000
}
    actual_noise_cols = [c for c in lidar_cols if c not in ['velocity_linear', 'velocity_angular']]
    wheels_noise_cols = [c for c in lidar_cols if c in ['encoder_left', 'encoder_right']]

    print(f"  Linear velocity: [{y_train[:,0].min():.3f}, {y_train[:,0].max():.3f}]")
    print(f"  Angular velocity: [{y_train[:,1].min():.3f}, {y_train[:,1].max():.3f}]")
    time.sleep(6)
    balanced_parts = []
    df = X_train_df.copy()
    df['velocity_linear'] = y_train[:, 0]
    df['velocity_angular'] = y_train[:, 1]
    print(y_train[5:1])
    print(df['velocity_angular'].head())
    np.random.seed(random_state)
    
    v = df['velocity_linear'].to_numpy()
    v_max = max(v.max() + 0.1, 1.5)
    bins = np.arange(0.0, v_max, 0.1)
    
    df['vel_bin'] = np.digitize(v, bins) - 1
    df['vel_bin'] = df['vel_bin'].clip(0, len(bins) - 2)
    df.loc[df['velocity_linear'] <= 0.0001, 'vel_bin'] = -1

    for bin_id, target_count in uniform.items():
        mask = df['vel_bin'] == bin_id
        bin_data = df[mask]
        
        if len(bin_data) == 0:
            print(f"Bin {bin_id}: Brak oryginalnych próbek. Dodano 0 (oczekiwano {target_count})")
            continue
            
        current_count = len(bin_data)
        
        is_oversampling = current_count < target_count
        action = "oversampled" if is_oversampling else "downsampled"

        bin_resampled = bin_data.sample(
            n=target_count, 
            replace=is_oversampling, 
            random_state=random_state
        )
        
        if LIDAR_NOISE and is_oversampling:
            
            bin_resampled_with_noise = bin_resampled.copy()
            duplicate_mask = bin_resampled_with_noise.index.duplicated(keep='first')
            
            if duplicate_mask.sum() > 0:
                std_dev = 0.001 
                num_duplicates = duplicate_mask.sum()
                num_lidar_cols = len(actual_noise_cols)

                noise_matrix = np.random.normal(0, std_dev, size=(num_duplicates, num_lidar_cols))
                
                bin_resampled_with_noise.loc[duplicate_mask, actual_noise_cols] += noise_matrix
                
                noisy_values = bin_resampled_with_noise.loc[duplicate_mask, actual_noise_cols].values
                bin_resampled_with_noise.loc[duplicate_mask, actual_noise_cols] = np.clip(noisy_values, None, None)
                
                action += " + noise"
            
            v_col = 'velocity_linear'
            bin_resampled_with_noise[v_col] = bin_resampled_with_noise[v_col].clip(0.0, 1.2)
            
            if bin_id == -1:
                bin_resampled_with_noise[v_col] = 0.0

            bin_resampled = bin_resampled_with_noise

        if WHEELS_NOISE and is_oversampling:
            bin_resampled_with_noise = bin_resampled.copy()
            duplicate_mask = bin_resampled_with_noise.index.duplicated(keep='first')
            
            if duplicate_mask.sum() > 0:
                std_dev = 0.0002 if bin_id == -1 else 0.002
                num_duplicates = duplicate_mask.sum()
                num_wheels_cols = len(wheels_noise_cols)

                noise_matrix = np.random.normal(0, std_dev, size=(num_duplicates, num_wheels_cols))
                
                bin_resampled_with_noise.loc[duplicate_mask, wheels_noise_cols] += noise_matrix
                
                action += " + noise"
            
            v_col = 'velocity_linear'
            bin_resampled_with_noise[v_col] = bin_resampled_with_noise[v_col].clip(0.0, 1.2)
            
            if bin_id == -1:
                bin_resampled_with_noise[v_col] = 0.0

            bin_resampled = bin_resampled_with_noise
            
        balanced_parts.append(bin_resampled)
        print(f"Bin {bin_id}: Po resamplingu i szumie mamy {len(bin_resampled)} próbek.")

    df_balanced = pd.concat(balanced_parts, ignore_index=True)
    df_balanced = df_balanced.sample(frac=1.0, random_state=random_state).reset_index(drop=True)
    df_balanced.loc[df_balanced['vel_bin'] == -1, 'velocity_linear'] = 0.0
    
    print(f"  Linear velocity: [{y_train[:,0].min():.3f}, {y_train[:,0].max():.3f}]")
    print(f"  Angular velocity: [{y_train[:,1].min():.3f}, {y_train[:,1].max():.3f}]")

    if ('velocity_linear' in df_balanced.columns) and ('velocity_angular' in df_balanced.columns):
        y_train = df_balanced[['velocity_linear', 'velocity_angular']].to_numpy(dtype=np.float32)
    else:
        raise RuntimeError("Balanced dataframe missing target columns 'velocity_linear'/'velocity_angular'")
    if (lidar_cols is not None) and (wheel_cols is not None):
        feature_columns = list(lidar_cols) + list(wheel_cols)
    else:
        feature_columns = [c for c in df_balanced.columns if c not in ('velocity_linear', 'velocity_angular', 'vel_bin')]
    X_train_df = df_balanced[feature_columns].reset_index(drop=True)

    print("Po przepisaniu:\n")
    print(f"  Linear velocity: [{y_train[:,0].min():.3f}, {y_train[:,0].max():.3f}]")
    print(f"  Angular velocity: [{y_train[:,1].min():.3f}, {y_train[:,1].max():.3f}]")

    if LIDAR_ZERO_ENABLED:
        X_train_df = apply_lidar_zero_pattern(X_train_df, lidar_cols, every_n=LIDAR_ZERO_EVERY_N, zero_len=LIDAR_ZERO_LEN, seed=LIDAR_ZERO_SEED)
        if X_test_df is not None:
            X_test_df = apply_lidar_zero_pattern(X_test_df, lidar_cols, every_n=LIDAR_ZERO_EVERY_N, zero_len=LIDAR_ZERO_LEN, seed=LIDAR_ZERO_SEED)

    if model_type == "cnn":
        if CNN_VARIANT == "lidar":
            X_train_df = make_cnn_lidar_only(X_train_df, lidar_cols)
        elif CNN_VARIANT == "lidar_wheels":
            X_train_df = make_cnn_lidar_wheels(X_train_df, lidar_cols, wheel_cols)
        
        if X_test_df is not None:
            if CNN_VARIANT == "lidar":
                X_test_df = make_cnn_lidar_only(X_test_df, lidar_cols)
            elif CNN_VARIANT == "lidar_wheels":
                X_test_df = make_cnn_lidar_wheels(X_test_df, lidar_cols, wheel_cols)

    if model_type == "gru" or model_type == "lstm":
        if GRU_VARIANT == "A1":
            X_train_df = make_input_A1_uniform(X_train_df, lidar_cols,wheel_cols)
            if X_test_df is not None:
                X_test_df = make_input_A1_uniform(X_test_df, lidar_cols, wheel_cols)
        elif GRU_VARIANT == "A2":
            X_train_df = make_input_A2_interleaved(X_train_df, lidar_cols,  wheel_cols)
            if X_test_df is not None:
                X_test_df = make_input_A2_interleaved(X_test_df, lidar_cols, wheel_cols)
        elif GRU_VARIANT == "B1":
            X_train_df = make_input_B1_uniform(X_train_df, lidar_cols,  wheel_cols)
            if X_test_df is not None:
                X_test_df = make_input_B1_uniform(X_test_df, lidar_cols, wheel_cols)
        elif GRU_VARIANT == "B2":
            X_train_df = make_input_B2_interleaved(X_train_df, lidar_cols,  wheel_cols)
            if X_test_df is not None:
                X_test_df = make_input_B2_interleaved(X_test_df, lidar_cols, wheel_cols)

    # del df_test
    gc.collect()

    print("Test velocity stats:")
    print(f"Mean: {y_test[:,0].mean():.3f}")
    print(f"Median: {np.median(y_test[:,0]):.3f}")
    print(f"Std: {y_test[:,0].std():.3f}")

    print("\n=== SCALING FOR TANH ===")
    print(f"Original ranges:")
    print(f"  Linear velocity: [{y_train[:,0].min():.3f}, {y_train[:,0].max():.3f}]")
    print(f"  Angular velocity: [{y_train[:,1].min():.3f}, {y_train[:,1].max():.3f}]")
    
    y_train_scaled = scale_outputs_for_tanh(y_train)
    print("Y_train_scaled:\n")
    print(y_train_scaled[:10])
    
    print(f"\nScaled to tanh range [-1, 1]:")
    print(f"  Linear velocity: [{y_train_scaled[:,0].min():.3f}, {y_train_scaled[:,0].max():.3f}]")
    print(f"  Angular velocity: [{y_train_scaled[:,1].min():.3f}, {y_train_scaled[:,1].max():.3f}]")
    print(f"  Input features: [{X_train_df.values.min():.3f}, {X_train_df.values.max():.3f}]")

    if y_test is not None:
        y_test_scaled = scale_outputs_for_tanh(y_test)
    else:
        y_test_scaled = None

    print("Y_test_scaled:\n")
    print(y_test_scaled[:10])
    print("Y_test:\n")
    print(y_test[:10])

    print("\n=== POST-SCALING VERIFICATION ===")
    print(f"y_train_scaled range: [{y_train_scaled[:,0].min():.3f}, {y_train_scaled[:,0].max():.3f}]")
    print(f"y_train_scaled mean: {y_train_scaled[:,0].mean():.3f}")
    print(f"y_train_scaled expected: close to 0.0")

    print(f"\n--- Showing input data distributions ---")
    # plot_data_distributions_before_training(y_train, y_test)

    X_train_norm = X_train_df
    X_test_norm = X_test_df

    if model_type == "cnn":

        if USE_DIFFS:
            X_train_final = X_train_norm 
            X_test_final  = X_test_norm
            y_test_final = y_test_scaled
            y_train_final = y_train_scaled 

            print("CNN + DIFFS shapes:", X_train_final.shape, y_train_final.shape)
            print("CNN + DIFFS + Z-SCORE shapes:", X_train_final.shape, y_train_final.shape)

        else:
            X_train_final = X_train_norm
            y_train_final = y_train_scaled

            if X_test_norm is not None:
                X_test_final = X_test_norm
                y_test_final = y_test_scaled

            print("CNN (no diffs) shapes:", X_train_final.shape, y_train_final.shape)

        X_train_final = np.expand_dims(X_train_final, axis=-1)
        if X_test_final is not None:
            X_test_final = np.expand_dims(X_test_final, axis=-1)

        print("Final CNN input:", X_train_final.shape, "y:", y_train_final.shape)
        if X_test_final is not None:
            print("Final CNN test:", X_test_final.shape, y_test_final.shape)


    if USE_SEQUENCE:
        X_train_final, y_train_final = create_sequences(X_train_norm, y_train_scaled, SEQUENCE_LENGTH)
        if X_test_norm is not None and y_test_scaled is not None:
            X_test_final, y_test_final = create_sequences(X_test_norm, y_test_scaled, SEQUENCE_LENGTH)
        print("After seq -> X_train:", X_train_norm.shape, " y_train:", y_train_scaled.shape)
        if X_test_norm is not None:
            print("After seq -> X_test:", X_test_norm.shape, " y_test:", y_test_scaled.shape)
        print(f"{'='*70}\n")

    if model_type in ["gru", "lstm"]:
        X_train_final = np.expand_dims(X_train_norm, axis=1)   # (N, 1, F)
        y_train_final = y_train_scaled

        if X_test_norm is not None:
            X_test_final = np.expand_dims(X_test_norm, axis=1) # (N, 1, F)
            y_test_final = y_test_scaled
    
    seed = random.seed()
    X_train_final, y_train_final = shuffle(X_train_final, y_train_final, random_state=seed)

    print(f"Model type: {model_type}")
    del X_train_norm, X_test_norm
    gc.collect()

    model, history = train_model(X_train_final, y_train_final, model_type)
    print(model.summary())
    model.save(model_save_path)
    if X_test_final is not None and y_test_final is not None:
        preds_scaled = model.predict(X_test_final, verbose=0)
        preds_unscaled = unscale_velocities_from_tanh(preds_scaled)
        y_test_unscaled = unscale_velocities_from_tanh(y_test_final)
        evaluate_and_plot_corrected(y_test_unscaled, preds_unscaled, history)
    gc.collect()
        
    print("\n=== Running final trajectory test ===")
    test_files = get_csv_files_from_paths(TEST_FOLDERS_TRAJ)
    df_traj = load_single_test_csv(TEST_FOLDERS_TRAJ[0])

    y_traj_odm = df_traj[["velocity_linear", "velocity_angular"]].to_numpy() 


    if len(test_files) > 0:
        X_test_df, y_traj = load_and_process_files(
            test_files,
            lidar_cols=lidar_cols,
            target_cols=["velocity_linear", "velocity_angular"],
            use_diffs=USE_DIFFS,
            normalize=True,
            wheel_cols=wheel_cols
        )
        all_feature_names = list(lidar_cols) + list(wheel_cols)
        X_test_df = pd.DataFrame(X_test_df, columns=all_feature_names)
    else:
        X_test_df, y_traj = None, None

    print("Ground truth")
    print(y_traj[:])

    calib = None
    src = getattr(df_traj, '_source_file', None) or df_traj.attrs.get('_source_file') if hasattr(df_traj, 'attrs') else None
    if src is not None:
        base = os.path.splitext(src)[0]
        calib_path = base + '.calib.json'
        if os.path.exists(calib_path):
            try:
                import json
                with open(calib_path, 'r') as fh:
                    calib = json.load(fh)
                print('Loaded calibration from', calib_path)
            except Exception:
                calib = None

    flip_param = None
    if calib is not None:
        if 'flip_left' in calib or 'flip_right' in calib:
            left_flag = bool(calib.get('flip_left', False))
            right_flag = bool(calib.get('flip_right', False))
            flip_param = (left_flag, right_flag)

    print(X_test_df[:5])
    y_traj_odom = extract_odometry_velocities_from_df(df_traj, wheel_cols, flip_sign=flip_param)
    if y_traj_odom is not None:
        print("Odom sample before sign-check:", y_traj_odom[:10])
        if calib is not None and calib.get('flip_x', False):
            try:
                print('Applying persisted calibration: flipping odom linear velocity sign (flip_x)')
                y_traj_odom[:, 0] = -y_traj_odom[:, 0]
            except Exception:
                pass

        try:
            corr = np.corrcoef(y_traj[:,0], y_traj_odom[:,0])[0,1]
            print(f"Odometry vs GT linear corr: {corr:.4f}")
            if (calib is None) and (corr < 0):
                print("Auto-flip: inverting odometry linear velocity sign to match GT")
                y_traj_odom[:,0] = -y_traj_odom[:,0]
        except Exception:
            pass
    print("Odom sample after sign-check:", y_traj_odom[:10])

    X_traj_df = X_test_df
    if not USE_DIFFS:
        X_traj_df = df_traj[lidar_cols + wheel_cols].copy()

    if model_type == "cnn":
        if CNN_VARIANT == "lidar":
            X_traj_df = make_cnn_lidar_only(X_traj_df, lidar_cols)
        elif CNN_VARIANT == "lidar_wheels":
            X_traj_df = make_cnn_lidar_wheels(X_traj_df, lidar_cols, wheel_cols)

    if model_type in ("gru", "lstm"):
        if GRU_VARIANT == "A1":
            X_traj_df = make_input_A1_uniform(X_traj_df, lidar_cols, wheel_cols)
        elif GRU_VARIANT == "A2":
            X_traj_df = make_input_A2_interleaved(X_traj_df, lidar_cols, wheel_cols)
        elif GRU_VARIANT == "B1":
            X_traj_df = make_input_B1_uniform(X_traj_df, lidar_cols, wheel_cols)
        elif GRU_VARIANT == "B2":
            X_traj_df = make_input_B2_interleaved(X_traj_df, lidar_cols, wheel_cols)

    X_traj_norm = X_traj_df
 
    Xr_traj = reshape_for_model(X_traj_norm, model_type)
    
    preds_traj_scaled = model.predict(Xr_traj)
    print(preds_traj_scaled[:])
    print(y_traj[:])
    try:
        print("Preds scaled range: min={:.4f}, max={:.4f}".format(np.nanmin(preds_traj_scaled), np.nanmax(preds_traj_scaled)))
    except Exception:
        print("Preds scaled (could not compute min/max)")

    preds_traj = unscale_velocities_from_tanh(preds_traj_scaled)
    try:
        print("Preds (unscaled) linear range: min={:.4f}, max={:.4f}".format(np.nanmin(preds_traj[:,0]), np.nanmax(preds_traj[:,0])))
    except Exception:
        print("Preds unscaled (could not compute min/max)")
    print("First 10 unscaled preds (linear, angular):")
    print(preds_traj[:])
    
    
    odom_transform = None
    if calib is not None:
        odom_transform = {
            'swap_axes': bool(calib.get('swap_axes', False)),
            'flip_x': bool(calib.get('flip_x', False)),
            'flip_y': bool(calib.get('flip_y', False)),
        }

    dist_gt = np.sum(np.abs(y_traj[:, 0])) * 0.1
    dist_pred = np.sum(np.abs(preds_traj[:, 0])) * 0.1
    dist_odom = np.sum(np.abs(y_traj_odom[:, 0])) * 0.1

    print(f"Droga Ground Truth: {dist_gt:.2f}m")
    print(f"Droga Predykcja:   {dist_pred:.2f}m")
    print(f"Droga Odometria:   {dist_odom:.2f}m")
    plot_three_trajectories(y_traj, y_traj_odom, preds_traj, dt=0.1, odom_transform=odom_transform)
    
    print("\n=== TRAINING COMPLETE ===")

def evaluate_and_plot_corrected(y_true_unscaled, preds_unscaled, history=None):
    y_true = np.asarray(y_true_unscaled)
    preds = np.asarray(preds_unscaled)

    if y_true.shape != preds.shape:
        min_n = min(len(y_true), len(preds))
        print(f"Shape mismatch: y_true {y_true.shape}, preds {preds.shape} -> obcinam do {min_n}")
        y_true = y_true[:min_n]
        preds = preds[:min_n]

    if len(y_true) == 0:
        print("No valid test data!")
        return

    errors = y_true - preds
    linear_errors = errors[:, 0]
    angular_errors = errors[:, 1]

    # Statistics
    print(f"\n{'='*70}")
    print(f"TEST SET EVALUATION")
    print(f"{'='*70}")
    print(f"Linear velocity:")
    print(f"  MAE:  {np.mean(np.abs(linear_errors)):.6f} m/s")
    print(f"  RMSE: {np.sqrt(np.mean(linear_errors**2)):.6f} m/s")
    print(f"  Mean error: {np.mean(linear_errors):.6f} m/s")
    print(f"  Std error:  {np.std(linear_errors):.6f} m/s")

    print(f"\nAngular velocity:")
    print(f"  MAE:  {np.mean(np.abs(angular_errors)):.6f} rad/s")
    print(f"  RMSE: {np.sqrt(np.mean(angular_errors**2)):.6f} rad/s")
    print(f"  Mean error: {np.mean(angular_errors):.6f} rad/s")
    print(f"  Std error:  {np.std(angular_errors):.6f} rad/s")
    print(f"{'='*70}\n")

    if history is not None:
        plot_training_curves(history)

    plot_error_histograms_with_gaussian(linear_errors, angular_errors)

    plot_velocity_timeseries(y_true, preds)

    print_per_bin_analysis(y_true, preds)

def print_per_bin_analysis(y_test, preds, bin_width=0.1):
    """Detailed per-bin error analysis"""
    print(f"\n{'='*100}")
    print(f"PER-BIN ERROR ANALYSIS (LINEAR VELOCITY)")
    print(f"{'='*100}")
    print(f"{'Range':^15} | {'N':^6} | {'%':^6} | {'Mean Real':^11} | {'Mean Pred':^11} | {'Mean Err':^11} | {'RMSE':^10} | {'MAE':^10}")
    print("-"*100)
    
    bins = np.arange(0, max(y_test[:,0].max(), preds[:,0].max()) + bin_width, bin_width)
    bin_idx = np.digitize(y_test[:,0], bins) - 1
    bin_idx = np.clip(bin_idx, 0, len(bins) - 2)
    
    total = len(y_test)
    
    for i in range(len(bins) - 1):
        mask = bin_idx == i
        if mask.sum() == 0:
            continue
        
        n = mask.sum()
        pct = 100 * n / total
        
        real = y_test[mask, 0]
        pred = preds[mask, 0]
        err = pred - real
        
        mean_real = real.mean()
        mean_pred = pred.mean()
        mean_err = err.mean()
        rmse = np.sqrt((err**2).mean())
        mae = np.abs(err).mean()
                
        print(f"[{bins[i]:4.1f}-{bins[i+1]:4.1f}] | {n:5d} | {pct:5.1f}% | {mean_real:10.4f} | {mean_pred:10.4f} | {mean_err:+10.4f} | {rmse:9.4f} | {mae:9.4f}")
    
    print("="*100 + "\n")

def plot_data_distributions_before_training(y_train, y_test):
    y_train_linear = y_train[:, 0] if y_train.ndim > 1 else y_train
    y_train_angular = y_train[:, 1] if y_train.ndim > 1 else np.zeros_like(y_train)
    y_test_linear = y_test[:, 0] if y_test.ndim > 1 else y_test
    y_test_angular = y_test[:, 1] if y_test.ndim > 1 else np.zeros_like(y_test)
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    bins = np.arange(LINEAR_VEL_MIN, LINEAR_VEL_MAX + 0.1, 0.1)
    axes[0, 0].hist(y_train_linear, bins=bins,  color='steelblue', edgecolor='black')
    axes[0, 0].set_title(f'Train: Linear Velocity (n={len(y_train_linear)})', fontweight='bold')
    axes[0, 0].set_xlabel('Linear Velocity [m/s]')
    axes[0, 0].set_ylabel('Count')
    axes[0, 0].grid(True, alpha=0.3)
    
    bins = np.arange(ANGULAR_VEL_MIN, ANGULAR_VEL_MAX + 0.1, 0.1)
    axes[0, 1].hist(y_train_angular, bins=bins, alpha=0.7, color='coral', edgecolor='black')
    axes[0, 1].set_title(f'Train: Angular Velocity (n={len(y_train_angular)})', fontweight='bold')
    axes[0, 1].set_xlabel('Angular Velocity [rad/s]')
    axes[0, 1].set_ylabel('Count')
    axes[0, 1].grid(True, alpha=0.3)
    
    bins = np.arange(LINEAR_VEL_MIN, LINEAR_VEL_MAX + 0.1, 0.1)
    axes[1, 0].hist(y_test_linear, bins=bins, color='mediumseagreen', edgecolor='black')
    axes[1, 0].set_title(f'Test: Linear Velocity (n={len(y_test_linear)})', fontweight='bold')
    axes[1, 0].set_xlabel('Linear Velocity [m/s]')
    axes[1, 0].set_ylabel('Count')
    axes[1, 0].grid(True, alpha=0.3)
    
    bins = np.arange(ANGULAR_VEL_MIN, ANGULAR_VEL_MAX + 0.1, 0.1)
    axes[1, 1].hist(y_test_angular, bins=bins, color='lightcoral', edgecolor='black')
    axes[1, 1].set_title(f'Test: Angular Velocity (n={len(y_test_angular)})', fontweight='bold')
    axes[1, 1].set_xlabel('Angular Velocity [rad/s]')
    axes[1, 1].set_ylabel('Count')
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.suptitle('Input Data Distribution (Before Training)', fontsize=14, fontweight='bold', y=1.00)
    plt.tight_layout()
    plt.show()


# def plot_training_curves(history):
#     """
#     Show 2 subplots: (1) loss and val_loss, (2) MSE and val_MSE.
#     """
#     fig, axes = plt.subplots(1, 4, figsize=(14, 5))
#     epochs = range(1, len(history.history['loss']) + 1)
    
#     # Loss subplot
#     axes[0].plot(epochs, history.history['loss'], 'o-', label='Training Loss', linewidth=2, markersize=3)
#     axes[0].plot(epochs, history.history['val_loss'], 's-', label='Validation Loss', linewidth=2, markersize=3)
#     axes[0].set_xlabel('Epoka')
#     axes[0].set_ylabel('Loss')
#     axes[0].legend(fontsize=10)
#     axes[0].grid(True, alpha=0.3)
    
#     # MSE subplot
#     if 'mse' in history.history:
#         axes[1].plot(epochs, history.history['mse'], 'o-', label='Training MSE', linewidth=2, markersize=3)
#         axes[1].plot(epochs, history.history['val_mse'], 's-', label='Validation MSE', linewidth=2, markersize=3)
#     else:
#         axes[1].plot(epochs, history.history['loss'], 'o-', label='Training Loss', linewidth=2, markersize=3)
#         axes[1].plot(epochs, history.history['val_loss'], 's-', label='Validation Loss', linewidth=2, markersize=3)
    
#     axes[1].set_xlabel('Epoka')
#     axes[1].set_ylabel('MSE')
#     axes[1].legend(fontsize=10)
#     axes[1].grid(True, alpha=0.3)

#     if 'mae' in history.history:
#         axes[2].plot(epochs, history.history['mae'], 'o-', label='Training MAE', linewidth=2, markersize=3)
#         axes[2].plot(epochs, history.history['val_mae'], 's-', label='Validation MAE', linewidth=2, markersize=3)
#     else:
#         axes[2].plot(epochs, history.history['loss'], 'o-', label='Training Loss', linewidth=2, markersize=3)
#         axes[2].plot(epochs, history.history['val_loss'], 's-', label='Validation Loss', linewidth=2, markersize=3)

#     axes[2].set_title('MAE', fontweight='bold', fontsize=12)
#     axes[2].set_xlabel('Epoch')
#     axes[2].set_ylabel('MAE')
#     axes[2].legend(fontsize=10)
#     axes[2].grid(True, alpha=0.3)

#     if 'accuracy' in history.history:
#         axes[3].plot(epochs, history.history['accuracy'], 'o-', label='Training Accuracy', linewidth=2, markersize=3)
#         axes[3].plot(epochs, history.history['val_accuracy'], 's-', label='Validation Accuracy', linewidth=2, markersize=3)
#     else:
#         axes[3].plot(epochs, history.history['loss'], 'o-', label='Training Loss', linewidth=2, markersize=3)
#         axes[3].plot(epochs, history.history['val_loss'], 's-', label='Validation Loss', linewidth=2, markersize=3)

#     axes[3].set_title('Accuracy', fontweight='bold', fontsize=12)
#     axes[3].set_xlabel('Epoch')
#     axes[3].set_ylabel('Accuracy')
#     axes[3].legend(fontsize=10)
#     axes[3].grid(True, alpha=0.3)

#     plt.tight_layout()
#     plt.show()

def plot_training_curves(history):
    epochs = range(1, len(history.history['loss']) + 1)
    
    metrics = [
        ('loss', 'Loss', 'Funkcja Straty (Loss)'),
        ('mse', 'MSE', ' MSE'),
        ('mae', 'MAE', 'MAE')
    ]

    for key, ylabel, title in metrics:
        if key in history.history:
            plt.figure(figsize=(8, 5))
            
            plt.plot(epochs, history.history[key], 'o-', label=f'Trening {ylabel}', 
                     linewidth=2, markersize=3)
            
            val_key = f'val_{key}'
            if val_key in history.history:
                plt.plot(epochs, history.history[val_key], 's-', label=f'Walidacja {ylabel}', 
                         linewidth=2, markersize=3)
            
            plt.xlabel('Epoka')
            plt.ylabel(ylabel)
            plt.legend(fontsize=10)
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.show()

# def plot_error_histograms_with_gaussian(errors_linear, errors_angular):
#     """
#     Show error distributions with Gaussian fits and μ, σ parameters.
#     """
#     fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
#     # Linear error histogram
#     mu_l, sigma_l = norm.fit(errors_linear)
#     x_lin = np.linspace(errors_linear.min(), errors_linear.max(), 200)
#     pdf_lin = norm.pdf(x_lin, mu_l, sigma_l)
    
#     axes[0].hist(errors_linear, bins=60, density=True, alpha=0.6, color='steelblue', edgecolor='black', label='Data')
#     axes[0].plot(x_lin, pdf_lin, 'r-', linewidth=2)
#     axes[0].axvline(mu_l, color='red', linestyle='--', linewidth=2, alpha=0.7)
#     axes[0].set_xlabel('Błąd estymacji [m/s]')
#     axes[0].set_ylabel('Gęstość')
#     axes[0].grid(True, alpha=0.3)
#     axes[0].text(0.98, 0.97, f'μ={mu_l:.4f}\nσ={sigma_l:.4f}', transform=axes[0].transAxes,
#                 ha='right', va='top', fontsize=10, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))
    
#     # Angular error histogram
#     mu_a, sigma_a = norm.fit(errors_angular)
#     x_ang = np.linspace(errors_angular.min(), errors_angular.max(), 200)
#     pdf_ang = norm.pdf(x_ang, mu_a, sigma_a)
    
#     axes[1].hist(errors_angular, bins=60, density=True, alpha=0.6, color='coral', edgecolor='black', label='Data')
#     axes[1].plot(x_ang, pdf_ang, 'r-', linewidth=2)
#     axes[1].axvline(mu_a, color='red', linestyle='--', linewidth=2, alpha=0.7)
#     axes[1].set_xlabel('Błąd estymacji [rad/s]')
#     axes[1].set_ylabel('Gęstość')
#     axes[1].grid(True, alpha=0.3)
#     axes[1].text(0.98, 0.97, f'μ={mu_a:.4f}\nσ={sigma_a:.4f}', transform=axes[1].transAxes,
#                 ha='right', va='top', fontsize=10, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))
    
#     plt.tight_layout()
#     plt.show()

def plot_error_histograms_with_gaussian(errors_linear, errors_angular):
    
    plt.figure(figsize=(8, 5))
    mu_l, sigma_l = norm.fit(errors_linear)
    x_lin = np.linspace(errors_linear.min(), errors_linear.max(), 200)
    pdf_lin = norm.pdf(x_lin, mu_l, sigma_l)
    
    plt.hist(errors_linear, bins=60, density=True, alpha=0.6, color='steelblue', edgecolor='black', label='Data')
    plt.plot(x_lin, pdf_lin, 'r-', linewidth=2)
    plt.axvline(mu_l, color='red', linestyle='--', linewidth=2, alpha=0.7)
    
    plt.xlabel('Błąd estymacji [m/s]')
    plt.ylabel('Gęstość')
    plt.grid(True, alpha=0.3)
    plt.text(0.98, 0.97, f'μ={mu_l:.4f}\nσ={sigma_l:.4f}', transform=plt.gca().transAxes,
                ha='right', va='top', fontsize=10, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))
    
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(8, 5))
    mu_a, sigma_a = norm.fit(errors_angular)
    x_ang = np.linspace(errors_angular.min(), errors_angular.max(), 200)
    pdf_ang = norm.pdf(x_ang, mu_a, sigma_a)
    
    plt.hist(errors_angular, bins=60, density=True, alpha=0.6, color='coral', edgecolor='black', label='Data')
    plt.plot(x_ang, pdf_ang, 'r-', linewidth=2)
    plt.axvline(mu_a, color='red', linestyle='--', linewidth=2, alpha=0.7)
    
    plt.xlabel('Błąd estymacji [rad/s]')
    plt.ylabel('Gęstość')
    plt.grid(True, alpha=0.3)
    plt.text(0.98, 0.97, f'μ={mu_a:.4f}\nσ={sigma_a:.4f}', transform=plt.gca().transAxes,
                ha='right', va='top', fontsize=10, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))
    
    plt.tight_layout()
    plt.show()


# def plot_velocity_timeseries(y_test, preds):
#     """
#     Show 2 time-series plots: (1) linear velocity (real vs pred), (2) angular velocity (real vs pred).
#     """
#     fig, axes = plt.subplots(2, 1, figsize=(15, 8))
#     t = np.arange(len(y_test))
    
#     # Linear velocity
#     axes[0].plot(t, y_test[:, 0], 'b-', label='Rzeczywista', linewidth=1.5, alpha=0.7)
#     axes[0].plot(t, preds[:, 0], 'r--', label='Predykcja', linewidth=1.5, alpha=0.7)
#     axes[0].set_xlabel('Próbka')
#     axes[0].set_ylabel('Prędkość liniowa [m/s]')
#     axes[0].legend(fontsize=10, loc='upper right')
#     axes[0].grid(True, alpha=0.3)
    
#     # Angular velocity
#     axes[1].plot(t, y_test[:, 1], 'b-', label='Rzeczywista', linewidth=1.5, alpha=0.7)
#     axes[1].plot(t, preds[:, 1], 'r--', label='Predykcja', linewidth=1.5, alpha=0.7)
#     axes[1].set_xlabel('Próbka')
#     axes[1].set_ylabel('Prędkość kątowa [rad/s]')
#     axes[1].legend(fontsize=10, loc='upper right')
#     axes[1].grid(True, alpha=0.3)
    
#     plt.tight_layout()
#     plt.show()

def plot_velocity_timeseries(y_test, preds):
    t = np.arange(len(y_test))
    
    plt.figure(figsize=(15, 5))
    plt.plot(t, y_test[:, 0], 'b-', label='Rzeczywista', linewidth=1.5, alpha=0.7)
    plt.plot(t, preds[:, 0], 'r--', label='Predykcja', linewidth=1.5, alpha=0.7)
    plt.xlabel('Próbka')
    plt.ylabel('Prędkość liniowa [m/s]')
    plt.legend(fontsize=10, loc='upper right')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()
    
    plt.figure(figsize=(15, 5))
    plt.plot(t, y_test[:, 1], 'b-', label='Rzeczywista', linewidth=1.5, alpha=0.7)
    plt.plot(t, preds[:, 1], 'r--', label='Predykcja', linewidth=1.5, alpha=0.7)
    plt.xlabel('Próbka')
    plt.ylabel('Prędkość kątowa [rad/s]')
    plt.legend(fontsize=10, loc='upper right')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


    

if __name__ == "__main__":
    main(TRAIN_FOLDERS, TEST_FOLDERS, MODEL_TYPE, MODEL_SAVE_PATH)