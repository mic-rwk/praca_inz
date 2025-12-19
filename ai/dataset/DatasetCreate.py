import argparse
import pandas as pd
import re
import ast
import os
import math

def parse_ros_string(s):
    data = {}

    # Encoder
    m = re.search(r"encoder_data=.*?left=([-\d\.e]+), right=([-\d\.e]+)", s)
    if m:
        data["encoder_left"] = float(m.group(1))
        data["encoder_right"] = float(m.group(2))

    # Laser
    m = re.search(r"laser_data=.*?ranges=\[(.+?)\]", s, re.DOTALL)
    if m:
        ranges_str = "[" + m.group(1) + "]"
        ranges_str = ranges_str.replace("inf", "float('inf')")
        try:
            data["laser_ranges"] = eval(ranges_str)
        except Exception as e:
            print("Błąd parsowania laser:", e)
            data["laser_ranges"] = []

    # Linear and angular velocity
    m = re.search(r"velocity_data=.*?linear=([-\d\.e]+), angular=([-\d\.e]+)", s)
    if m:
        data["velocity_linear"] = float(m.group(1))
        data["velocity_angular"] = float(m.group(2))
    
    # Diff laser
    m = re.search(r"diff_laser_data=.*?ranges=\[(.+?)\]", s, re.DOTALL)
    if m:
        ranges_str = "[" + m.group(1) + "]"
        ranges_str = ranges_str.replace("nan", "float('nan')").replace("inf", "math.inf")
        try:
            data["diff_laser_ranges"] = eval(ranges_str)
        except Exception as e:
            print("Błąd parsowania diff laser:", e)
            data["diff_laser_ranges"] = []

    return data

def extract_data(source_file_path, options=("laser", "encoder", "velocity", "diff_laser")):
    df = pd.read_csv(source_file_path, header=None, names=["Timestamp", "Data"])

    records = []
    for _, row in df.iterrows():
        parsed = parse_ros_string(row["Data"])
        rec = {"Timestamp": row["Timestamp"]}

        rec["velocity_linear"] = parsed.get("velocity_linear") if "velocity" in options else None
        rec["velocity_angular"] = parsed.get("velocity_angular") if "velocity" in options else None
        
        rec["encoder_left"] = parsed.get("encoder_left") if "encoder" in options else "nil"
        rec["encoder_right"] = parsed.get("encoder_right") if "encoder" in options else "nil"

        lidar_key = None
        if "diff_laser" in options and "diff_laser_ranges" in parsed:
            lidar_key = "diff_laser_ranges"
        elif "laser" in options and "laser_ranges" in parsed:
            lidar_key = "laser_ranges"

        if lidar_key:
            lidar_data = parsed.get(lidar_key)
            for i, value in enumerate(lidar_data):
                rec[f"lidar_{i+1}"] = value

        records.append(rec)

    result_df = pd.DataFrame(records)

    # Some robot firmware may publish cumulative encoder counts instead of
    # instantaneous velocities. Convert to delta (instantaneous) by differencing.
    if 'encoder_left' in result_df.columns and 'encoder_right' in result_df.columns:
        # Ensure numeric (coerce 'nil' or bad strings to NaN)
        result_df['encoder_left'] = pd.to_numeric(result_df['encoder_left'], errors='coerce')
        result_df['encoder_right'] = pd.to_numeric(result_df['encoder_right'], errors='coerce')

        # Differentiate cumulative encoder values to obtain per-sample increments
        result_df['encoder_left'] = result_df['encoder_left'].diff().fillna(0)
        result_df['encoder_right'] = result_df['encoder_right'].diff().fillna(0)

    return result_df

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        prog='DatasetCreate',
        description='Convert CSV that contains rosbag output to CSV with bare information for NN'
    )
    parser.add_argument("-f", "--filename")
    
    args = parser.parse_args()

    script_directory = os.path.dirname(os.path.abspath(__file__))

    project_root = os.path.abspath(os.path.join(script_directory, ".."))
    source_file = os.path.join(project_root, args.filename) if not os.path.isabs(args.filename) else args.filename

    csv_output = "./csv_output/"

    # df_laser = extract_data(source_file, options=("laser", "velocity"))
    # df_laser.to_csv(f"{csv_output}laser.csv", index=False)

    df_combo = extract_data(source_file, options=("laser", "encoder", "velocity"))
    df_combo.to_csv(f"{csv_output}laser_encoder.csv", index=False)

    # df_diff_laser = extract_data(source_file, options=("diff_laser", "velocity"))
    # df_diff_laser.to_csv(f"{csv_output}diff_laser.csv", index=False)

    # df_diff_encoder = extract_data(source_file, options=("diff_laser", "encoder", "velocity"))
    # df_diff_encoder.to_csv(f"{csv_output}diff_laser_encoder.csv", index=False)