import argparse
import pandas as pd
import re
import ast
import os
import math
import glob

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
    # Zmieniamy argument -f, aby obsługiwał wzorce (np. "*2026*")
    parser.add_argument("-f", "--filename", help="Nazwa pliku lub wzorzec (np. '*2026*.csv')")
    # Dodajemy argument dla ścieżki wyjściowej
    parser.add_argument("-o", "--output", default="./csv_output/", help="Folder docelowy dla plików CSV")
    
    args = parser.parse_args()

    # Tworzenie folderu wyjściowego, jeśli nie istnieje
    if not os.path.exists(args.output):
        os.makedirs(args.output)
        print(f"Utworzono folder: {args.output}")

    # Znajdowanie wszystkich plików pasujących do wzorca
    files = glob.glob(args.filename)

    if not files:
        print(f"Nie znaleziono plików pasujących do wzorca: {args.filename}")
    else:
        for file_path in files:
            print(f"Przetwarzam: {file_path}")
            
            # Pobieramy nazwę pliku bez rozszerzenia do stworzenia unikalnej nazwy wyjściowej
            base_name = os.path.splitext(os.path.basename(file_path))[0]
            
            # Wywołanie Twojej funkcji extract_data
            df_combo = extract_data(file_path, options=("laser", "encoder", "velocity"))
            
            # Dynamiczna nazwa pliku wyjściowego
            output_file = os.path.join(args.output, f"laser_encoder_{base_name}.csv")
            
            df_combo.to_csv(output_file, index=False)
            print(f"Zapisano do: {output_file}")