import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict

# --- parametry konfiguracyjne ---
# Prędkość liniowa
PROCESS_LINEAR = True
LINEAR_BIN_SIZE = 0.1
LINEAR_MAX_VEL = 1.2
LINEAR_TARGET_PER_BIN = 1100
LINEAR_MIN_PER_BIN = 1000      
LINEAR_MAX_PER_BIN = 2700

# Prędkość kątowa
PROCESS_ANGULAR = True
ANGULAR_BIN_SIZE = 0.1
ANGULAR_MAX_VEL = 3.0
ANGULAR_TARGET_PER_BIN = 600
ANGULAR_MIN_PER_BIN = 400
ANGULAR_MAX_PER_BIN = 1200

# Wspólne
MIN_VEL_FOR_ZERO = 0.01
VELOCITY_COL = "velocity_linear"
ANGULAR_COL = "velocity_angular"

# Opcje balansowania
AUTO_BALANCE = True
BALANCE_THRESHOLD = 0.3
MULTI_PASS = True

FLEXIBLE_MAP_LIMITS = True  # mapy mogą przekroczyć TARGET do MAX
STRICT_BIN_LIMITS = False   # nie przekraczaj MAX_PER_BIN (ścisły limit)

HIGH_PRIORITY_LINEAR_RANGES = [(0.5, 1.2, 9.0),
                               (0.6, 1.0, 15.0)]
HHIGH_PRIORITY_ANGULAR_RANGES = [
    (-2.0, -1.0, 8.0),
    (0.0, 2.0, 10.0)
]

ANGULAR_DIVERSITY_BONUS = True
PENALIZE_LOW_ANGULAR = True

# If True, reject any 2-row pair that contains per-row velocities outside the
# configured per-row limits (safer than clipping). Set to False to allow
# clipping fallback instead.
STRICT_PAIR_CHECK = True
def analyze_distribution(folders, option):
    """
    Pierwszy przebieg: analiza rozkładu danych
    Zwraca statystyki binów dla wszystkich map
    """
    linear_stats = defaultdict(int)
    angular_stats = defaultdict(int)
    
    for folder in folders:
        files = sorted(glob.glob(os.path.join(folder, "*.csv")))
        for f in files:
            try:
                df = pd.read_csv(f)
                df[VELOCITY_COL] = pd.to_numeric(df[VELOCITY_COL], errors="coerce").fillna(0)
                df[ANGULAR_COL] = pd.to_numeric(df.get(ANGULAR_COL, 0), errors="coerce").fillna(0)
                
                df.loc[df[VELOCITY_COL].abs() < MIN_VEL_FOR_ZERO, VELOCITY_COL] = 0
                df.loc[df[ANGULAR_COL].abs() < MIN_VEL_FOR_ZERO, ANGULAR_COL] = 0
                
                # Zlicz w binach
                if PROCESS_LINEAR:
                    linear_vals = df[VELOCITY_COL].abs()
                    for v in linear_vals:
                        if v <= LINEAR_MAX_VEL:
                            bin_idx = int(v / LINEAR_BIN_SIZE)
                            linear_stats[bin_idx] += 1
                
                if PROCESS_ANGULAR:
                    angular_vals = df[ANGULAR_COL]
                    for w in angular_vals:
                        if abs(w) <= ANGULAR_MAX_VEL:
                            bin_idx = int(abs(w) / ANGULAR_BIN_SIZE)
                            angular_stats[bin_idx] += 1
                            
            except Exception as e:
                continue
    
    return linear_stats, angular_stats


def print_distribution_analysis(linear_stats, angular_stats, num_maps):
    """Wyświetla analizę rozkładu i rekomendacje"""
    print("\n" + "="*70)
    print("ANALIZA ROZKŁADU DANYCH")
    print("="*70)
    
    if PROCESS_LINEAR and linear_stats:
        print(f"\nPrędkość liniowa (cel: {LINEAR_TARGET_PER_BIN * num_maps} na bin łącznie):")
        underrep_linear = []
        overrep_linear = []
        
        for bin_idx in sorted(linear_stats.keys()):
            v_min = bin_idx * LINEAR_BIN_SIZE
            v_max = (bin_idx + 1) * LINEAR_BIN_SIZE
            count = linear_stats[bin_idx]
            target = LINEAR_TARGET_PER_BIN * num_maps
            percentage = (count / target) * 100 if target > 0 else 0
            
            if percentage < BALANCE_THRESHOLD * 100:
                status = "[ERR] NIEDOREPREZENTOWANY"
                underrep_linear.append((v_min, v_max))
            elif percentage > 150:
                status = " NADREPREZENTOWANY"
                overrep_linear.append((v_min, v_max))
            else:
                status = "✓ OK"
            
            print(f"  [{v_min:.1f}-{v_max:.1f}] m/s: {count:5d} ({percentage:6.1f}%) {status}")
        
        if underrep_linear:
            print(f"\n  💡 Sugestia: Ustaw UNLIMITED_LINEAR_RANGES na niedoreprezentowane zakresy")
        if overrep_linear:
            print(f"  💡 Nadreprezentowane biny zostaną ograniczone do {LINEAR_TARGET_PER_BIN}/mapę")
    
    if PROCESS_ANGULAR and angular_stats:
        print(f"\nPrędkość kątowa (cel: {ANGULAR_TARGET_PER_BIN * num_maps} na bin łącznie):")
        underrep_angular = []
        overrep_angular = []
        
        for bin_idx in sorted(angular_stats.keys()):
            w_min = bin_idx * ANGULAR_BIN_SIZE
            w_max = (bin_idx + 1) * ANGULAR_BIN_SIZE
            count = angular_stats[bin_idx]
            target = ANGULAR_TARGET_PER_BIN * num_maps
            percentage = (count / target) * 100 if target > 0 else 0
            
            if percentage < BALANCE_THRESHOLD * 100:
                status = "[ERR] NIEDOREPREZENTOWANY"
                underrep_angular.append((w_min, w_max))
            elif percentage > 150:
                status = " NADREPREZENTOWANY"
                overrep_angular.append((w_min, w_max))
            else:
                status = "✓ OK"
            
            print(f"  [{w_min:.1f}-{w_max:.1f}] rad/s: {count:5d} ({percentage:6.1f}%) {status}")
        
        if underrep_angular:
            print(f"\n  💡 Sugestia: Ustaw UNLIMITED_ANGULAR_RANGES na niedoreprezentowane zakresy")


def get_priority_bins(stats, target_per_bin, num_maps, threshold=BALANCE_THRESHOLD):
    """
    Zwraca listę binów, które mają priorytet (są niedoreprezentowane)
    """
    priority = set()
    total_target = target_per_bin * num_maps
    
    for bin_idx, count in stats.items():
        if count < total_target * threshold:
            priority.add(bin_idx)
    
    return priority


def is_unlimited(value, ranges):
    for rmin, rmax in ranges:
        if rmin <= value <= rmax:
            return True
    return False


def make_pairs(df):
    """Zwraca listę 2-wierszowych par: (0,1),(2,3)…"""
    pairs = []
    for i in range(0, len(df) - 1, 2):
        pairs.append(df.iloc[i:i + 2])
    return pairs


def filter_stationary_duplicates(df, vel_col=VELOCITY_COL, ang_col=ANGULAR_COL, rot_threshold=0.1):
    """
    Usuwa kolejne próbki z zerową prędkością liniową, jeśli robot stoi i się nie obraca.
    """
    filtered_rows = []
    was_stationary = False

    for idx, row in df.iterrows():
        v = abs(row[vel_col])
        w = abs(row[ang_col])

        if v == 0 and w < rot_threshold:
            if not was_stationary:
                filtered_rows.append(row)
                was_stationary = True
        else:
            filtered_rows.append(row)
            was_stationary = False

    result_df = pd.DataFrame(filtered_rows, columns=df.columns)
    return result_df.reset_index(drop=True)


def get_bin_limit(velocity, is_linear=True):
    """
    Zwraca limit dla danego binu w zależności od konfiguracji.
    Pozwala na różne limity dla różnych zakresów prędkości.
    """
    if is_linear:
        base_limit = LINEAR_TARGET_PER_BIN
        max_limit = LINEAR_MAX_PER_BIN
        priority_ranges = HIGH_PRIORITY_LINEAR_RANGES
    else:
        base_limit = ANGULAR_TARGET_PER_BIN
        max_limit = ANGULAR_MAX_PER_BIN
        priority_ranges = HHIGH_PRIORITY_ANGULAR_RANGES
    
    # Sprawdź czy prędkość jest w zakresie priorytetowym
    multiplier = 1.0
    for min_v, max_v, mult in priority_ranges:
        if min_v <= velocity <= max_v:
            multiplier = mult
            break
    
    # Oblicz limit
    limit = int(base_limit * multiplier)
    
    # Nie przekraczaj maksimum jeśli STRICT_BIN_LIMITS
    if STRICT_BIN_LIMITS:
        limit = min(limit, max_limit)
    elif FLEXIBLE_MAP_LIMITS:
        # Pozwól na większy limit, ale nie więcej niż MAX
        limit = min(limit, max_limit)
    
    return limit


def calculate_priority_score(linear_v, angular_v, linear_bin_limits, angular_bin_limits):
    """
    Oblicza priorytet pary (linear, angular).
    Wyższy wynik = wyższy priorytet.
    
    Priorytet oparty na:
    1. Deficyt binu (jak daleko od celu)
    2. Różnorodność kątowa (preferuj różne wartości angular dla tej samej linear)
    3. Kara dla nadreprezentowanych zakresów
    """
    score = 0
    
    # === PRĘDKOŚĆ LINIOWA ===
    linear_bin = int(linear_v / LINEAR_BIN_SIZE)
    current_linear_count = linear_bin_limits.get(linear_bin, 0)
    linear_limit = get_bin_limit(linear_v, is_linear=True)
    linear_deficit = LINEAR_MIN_PER_BIN - current_linear_count
    
    if linear_deficit > 0:
        # Biny poniżej minimum mają bardzo wysoki priorytet
        score += linear_deficit * 10
    else:
        # Powyżej minimum, ale poniżej celu/limitu
        target_deficit = linear_limit - current_linear_count
        if target_deficit > 0:
            score += target_deficit * 3  # Zwiększone z 2 na 3
    
    # === PRĘDKOŚĆ KĄTOWA ===
    angular_bin = int(abs(angular_v) / ANGULAR_BIN_SIZE)
    current_angular_count = angular_bin_limits.get(angular_bin, 0)
    angular_limit = get_bin_limit(abs(angular_v), is_linear=False)
    angular_deficit = ANGULAR_MIN_PER_BIN - current_angular_count
    
    if angular_deficit > 0:
        score += angular_deficit * 10
    else:
        target_deficit = angular_limit - current_angular_count
        if target_deficit > 0:
            score += target_deficit * 3  # Zwiększone z 2 na 3
    
    # === BONUS ZA RÓŻNORODNOŚĆ KĄTOWĄ ===
    if ANGULAR_DIVERSITY_BONUS:
        angular_abs = abs(angular_v)
        
        # Bardzo wysoki bonus dla ekstremalnych prędkości kątowych (>1.0 rad/s)
        if angular_abs > 1.0:
            score += 500 + (angular_abs * 200)
        elif angular_abs > 0.5:
            score += 300 + (angular_abs * 100)
        elif angular_abs > 0.3:
            score += 150
    
    # === KARA dla nadreprezentowanych niskich prędkości kątowych ===
    if PENALIZE_LOW_ANGULAR:
        angular_abs = abs(angular_v)
        if angular_abs < 0.2:
            # Sprawdź czy ten bin jest już bardzo pełny
            if current_angular_count > ANGULAR_TARGET_PER_BIN * 0.8:
                score -= 1000  # Duża kara dla prawie pełnych binów niskiej prędkości
            elif current_angular_count > ANGULAR_MIN_PER_BIN:
                score -= 300   # Średnia kara
    
    # === BONUS dla wysokich prędkości liniowych (mniejszy niż poprzednio) ===
    if linear_v >= 0.6:
        score += 100 + (linear_v * 30)
    elif linear_v >= 0.4:
        score += 50
    
    # === BONUS dla kombinacji: wysoka linear + ekstremalna angular ===
    if linear_v >= 0.5 and abs(angular_v) > 1.0:
        score += 400  # Bardzo cenne dane!
    
    return score


def process_map(folder, output_root="processed", priority_linear=None, priority_angular=None):
    """
    Procesuje wszystkie pliki jednej mapy z wieloprzebiegowym balansowaniem
    """
    files = sorted(glob.glob(os.path.join(folder, "*.csv")))
    if not files:
        print(f"Brak plików w {folder}")
        return None, None

    map_name = os.path.basename(os.path.dirname(os.path.normpath(folder)))
    out_dir = os.path.join(output_root, map_name)
    os.makedirs(out_dir, exist_ok=True)

    raw_all = []
    proc_all = []
    
    linear_bin_limits = defaultdict(int)
    angular_bin_limits = defaultdict(int)

    # Wczytaj wszystkie pary
    all_pairs = []
    for f in files:
        try:
            raw = pd.read_csv(f)
            raw_all.append(raw)
            
            df = raw.copy()
            df[VELOCITY_COL] = pd.to_numeric(df[VELOCITY_COL], errors="coerce").fillna(0)
            df[ANGULAR_COL] = pd.to_numeric(df.get(ANGULAR_COL, 0), errors="coerce").fillna(0)
            
            # Zero very small velocities and force non-negative linear speeds
            df.loc[df[VELOCITY_COL].abs() < MIN_VEL_FOR_ZERO, VELOCITY_COL] = 0
            df.loc[df[ANGULAR_COL].abs() < MIN_VEL_FOR_ZERO, ANGULAR_COL] = 0
            # Ensure linear velocity is non-negative (no negative values at all)
            df[VELOCITY_COL] = df[VELOCITY_COL].where(df[VELOCITY_COL] >= 0.0, 0.0)
            df = filter_stationary_duplicates(df)
            
            pairs = make_pairs(df)
            for idx, pair in enumerate(pairs):
                # Per-row checks: ensure each member of the pair respects per-row limits
                pair_lin_vals = pair[VELOCITY_COL].to_numpy(dtype=float)
                pair_ang_vals = pair[ANGULAR_COL].to_numpy(dtype=float)

                # If strict checking is enabled, skip pairs that contain any out-of-range row
                if STRICT_PAIR_CHECK:
                    if (pair_lin_vals < 0.0).any() or (pair_lin_vals > LINEAR_MAX_VEL).any() or \
                       (np.abs(pair_ang_vals) > ANGULAR_MAX_VEL).any():
                        # skip this pair entirely
                        continue

                linear_v = float(np.abs(pair_lin_vals).mean())
                angular_v = float(pair_ang_vals.mean())
                # Używamy unikalnego ID zamiast porównywania DataFrame'ów
                pair_id = (f, idx)
                all_pairs.append((linear_v, angular_v, pair, f, pair_id))
                    
        except Exception as e:
            print(f"Błąd wczytywania {f}: {e}")
            continue
    
    print(f"  Zebrano {len(all_pairs)} par do przetworzenia")
    
    accepted_by_file = defaultdict(list)
    accepted_pair_ids = set()  # Śledzenie dodanych par
    rejected_count = 0
    
    if MULTI_PASS:
        # PRZEBIEG 1: Wypełnij wszystkie biny do minimum
        print(f"  PRZEBIEG 1: Wypełnianie do minimum ({LINEAR_MIN_PER_BIN}/{ANGULAR_MIN_PER_BIN})")
        
        for iteration in range(5):  # Zwiększono do 5 iteracji
            # Przelicz priorytety na podstawie aktualnego stanu
            all_pairs_with_priority = []
            for linear_v, angular_v, pair, filename, pair_id in all_pairs:
                # Pomiń już dodane pary
                if pair_id in accepted_pair_ids:
                    continue
                    
                priority = calculate_priority_score(linear_v, angular_v, linear_bin_limits, angular_bin_limits)
                all_pairs_with_priority.append((priority, linear_v, angular_v, pair, filename, pair_id))
            
            # Sortuj po priorytecie
            all_pairs_with_priority.sort(key=lambda x: x[0], reverse=True)
            
            added_this_iteration = 0
            
            for priority, linear_v, angular_v, pair, filename, pair_id in all_pairs_with_priority:
                if priority <= 0:  # Wszystkie biny osiągnęły minimum
                    break
                
                accept = True
                
                # Sprawdź limity
                if PROCESS_LINEAR:
                    if linear_v > LINEAR_MAX_VEL:
                        accept = False
                    else:
                        linear_bin = int(linear_v / LINEAR_BIN_SIZE)
                        # W pierwszym przebiegu: ścisły limit na minimum
                        if linear_bin_limits[linear_bin] + 2 > LINEAR_MIN_PER_BIN:
                            accept = False
                
                if PROCESS_ANGULAR and accept:
                    if abs(angular_v) > ANGULAR_MAX_VEL:
                        accept = False
                    else:
                        angular_bin = int(abs(angular_v) / ANGULAR_BIN_SIZE)
                        if angular_bin_limits[angular_bin] + 2 > ANGULAR_MIN_PER_BIN:
                            accept = False
                
                if accept:
                    accepted_by_file[filename].append(pair)
                    accepted_pair_ids.add(pair_id)
                    if PROCESS_LINEAR:
                        linear_bin = int(linear_v / LINEAR_BIN_SIZE)
                        linear_bin_limits[linear_bin] += 2
                    if PROCESS_ANGULAR:
                        angular_bin = int(abs(angular_v) / ANGULAR_BIN_SIZE)
                        angular_bin_limits[angular_bin] += 2
                    added_this_iteration += 2
            
            print(f"    Iteracja {iteration + 1}: dodano {added_this_iteration} rekordów")
            
            if added_this_iteration == 0:
                break
        
        # PRZEBIEG 2: Dopełnij do celu/limitu (z uwzględnieniem priorytetowych zakresów)
        print(f"  PRZEBIEG 2: Dopełnianie do docelowych limitów")
        if FLEXIBLE_MAP_LIMITS:
            print(f"    Tryb elastyczny: mapy mogą przekroczyć TARGET do MAX")
        if HIGH_PRIORITY_LINEAR_RANGES:
            print(f"    Zakresy priorytetowe linear: {HIGH_PRIORITY_LINEAR_RANGES}")
        if HHIGH_PRIORITY_ANGULAR_RANGES:
            print(f"    Zakresy priorytetowe angular: {HHIGH_PRIORITY_ANGULAR_RANGES}")
        
        # Wielokrotne iteracje dla lepszego wypełnienia
        for iteration in range(3):
            # Przelicz priorytety ponownie
            all_pairs_with_priority = []
            for linear_v, angular_v, pair, filename, pair_id in all_pairs:
                # Pomiń już dodane pary
                if pair_id in accepted_pair_ids:
                    continue
                    
                priority = calculate_priority_score(linear_v, angular_v, linear_bin_limits, angular_bin_limits)
                all_pairs_with_priority.append((priority, linear_v, angular_v, pair, filename, pair_id))
            
            all_pairs_with_priority.sort(key=lambda x: x[0], reverse=True)
            
            added_this_iteration = 0
            
            for priority, linear_v, angular_v, pair, filename, pair_id in all_pairs_with_priority:
                if priority <= 0:
                    break
                    
                accept = True
                
                if PROCESS_LINEAR:
                    if linear_v > LINEAR_MAX_VEL:
                        accept = False
                    else:
                        linear_bin = int(linear_v / LINEAR_BIN_SIZE)
                        linear_limit = get_bin_limit(linear_v, is_linear=True)
                        
                        if linear_bin_limits[linear_bin] + 2 > linear_limit:
                            accept = False
                
                if PROCESS_ANGULAR and accept:
                    if abs(angular_v) > ANGULAR_MAX_VEL:
                        accept = False
                    else:
                        angular_bin = int(abs(angular_v) / ANGULAR_BIN_SIZE)
                        angular_limit = get_bin_limit(abs(angular_v), is_linear=False)
                        
                        if angular_bin_limits[angular_bin] + 2 > angular_limit:
                            accept = False
                
                if accept:
                    accepted_by_file[filename].append(pair)
                    accepted_pair_ids.add(pair_id)
                    if PROCESS_LINEAR:
                        linear_bin = int(linear_v / LINEAR_BIN_SIZE)
                        linear_bin_limits[linear_bin] += 2
                    if PROCESS_ANGULAR:
                        angular_bin = int(abs(angular_v) / ANGULAR_BIN_SIZE)
                        angular_bin_limits[angular_bin] += 2
                    added_this_iteration += 2
                else:
                    rejected_count += 1
            
            print(f"    Iteracja {iteration + 1}: dodano {added_this_iteration} rekordów")
            
            if added_this_iteration == 0:
                break
    
    total_accepted = sum(len(pairs) for pairs in accepted_by_file.values())
    print(f"  Zaakceptowano: {total_accepted}, Odrzucono: {len(all_pairs) * 2 - total_accepted}")
    
    # Zapisz przetworzone pliki
    for filename, pairs_list in accepted_by_file.items():
        if pairs_list:
            processed = pd.concat(pairs_list, ignore_index=True)
            # Clip processed rows to allowed velocity ranges so individual samples
            # cannot exceed configured limits even if the pair mean was acceptable.
            if VELOCITY_COL in processed.columns:
                # Ensure linear velocities are within [0, LINEAR_MAX_VEL]
                before_min = processed[VELOCITY_COL].min()
                before_max = processed[VELOCITY_COL].max()
                processed[VELOCITY_COL] = processed[VELOCITY_COL].clip(lower=0.0, upper=LINEAR_MAX_VEL)
                after_min = processed[VELOCITY_COL].min()
                after_max = processed[VELOCITY_COL].max()
                if before_min != after_min or before_max != after_max:
                    print(f"    Clipped linear velocities in {os.path.basename(filename)}: [{before_min:.3f},{before_max:.3f}] -> [{after_min:.3f},{after_max:.3f}]")
            if ANGULAR_COL in processed.columns:
                # Ensure angular velocities are within [-ANGULAR_MAX_VEL, ANGULAR_MAX_VEL]
                before_min_a = processed[ANGULAR_COL].min()
                before_max_a = processed[ANGULAR_COL].max()
                processed[ANGULAR_COL] = processed[ANGULAR_COL].clip(lower=-ANGULAR_MAX_VEL, upper=ANGULAR_MAX_VEL)
                after_min_a = processed[ANGULAR_COL].min()
                after_max_a = processed[ANGULAR_COL].max()
                if before_min_a != after_min_a or before_max_a != after_max_a:
                    print(f"    Clipped angular velocities in {os.path.basename(filename)}: [{before_min_a:.3f},{before_max_a:.3f}] -> [{after_min_a:.3f},{after_max_a:.3f}]")

            proc_all.append(processed)
            out_path = os.path.join(out_dir, os.path.basename(filename))
            processed.to_csv(out_path, index=False)

    # Statystyki binów
    print(f"\nStatystyki binów dla mapy {map_name}:")
    
    if PROCESS_LINEAR:
        print(f"  Prędkość liniowa (min: {LINEAR_MIN_PER_BIN}, target: {LINEAR_TARGET_PER_BIN}, max: {LINEAR_MAX_PER_BIN}):")
        for bin_idx in sorted(linear_bin_limits.keys()):
            v_min = bin_idx * LINEAR_BIN_SIZE
            v_max = (bin_idx + 1) * LINEAR_BIN_SIZE
            count = linear_bin_limits[bin_idx]
            
            # Oblicz limit dla tego binu
            bin_center = (v_min + v_max) / 2
            bin_limit = get_bin_limit(bin_center, is_linear=True)
            percentage = (count / bin_limit) * 100
            
            if count < LINEAR_MIN_PER_BIN:
                status = "[ERR]"
            elif count >= bin_limit * 0.9:
                status = "✓"
            else:
                status = "◌"
            
            # Pokaż limit jeśli różny od TARGET
            if bin_limit != LINEAR_TARGET_PER_BIN:
                print(f"    {status} Bin [{v_min:.1f}-{v_max:.1f}] m/s: {count:4d}/{bin_limit} ({percentage:5.1f}%)")
            else:
                print(f"    {status} Bin [{v_min:.1f}-{v_max:.1f}] m/s: {count:4d} ({percentage:5.1f}%)")
    
    if PROCESS_ANGULAR:
        print(f"  Prędkość kątowa (min: {ANGULAR_MIN_PER_BIN}, target: {ANGULAR_TARGET_PER_BIN}, max: {ANGULAR_MAX_PER_BIN}):")
        for bin_idx in sorted(angular_bin_limits.keys()):
            w_min = bin_idx * ANGULAR_BIN_SIZE
            w_max = (bin_idx + 1) * ANGULAR_BIN_SIZE
            count = angular_bin_limits[bin_idx]
            
            # Oblicz limit dla tego binu
            bin_center = (w_min + w_max) / 2
            bin_limit = get_bin_limit(bin_center, is_linear=False)
            percentage = (count / bin_limit) * 100
            
            if count < ANGULAR_MIN_PER_BIN:
                status = "[ERR]"
            elif count >= bin_limit * 0.9:
                status = "✓"
            else:
                status = "◌"
            
            # Pokaż limit jeśli różny od TARGET
            if bin_limit != ANGULAR_TARGET_PER_BIN:
                print(f"    {status} Bin [{w_min:.1f}-{w_max:.1f}] rad/s: {count:4d}/{bin_limit} ({percentage:5.1f}%)")
            else:
                print(f"    {status} Bin [{w_min:.1f}-{w_max:.1f}] rad/s: {count:4d} ({percentage:5.1f}%)")

    raw_combined = pd.concat(raw_all, ignore_index=True) if raw_all else None
    proc_combined = pd.concat(proc_all, ignore_index=True) if proc_all else None
    
    return raw_combined, proc_combined


def plot_histogram(raw, proc, map_name):
    linear_bins = np.arange(0, LINEAR_MAX_VEL + LINEAR_BIN_SIZE, LINEAR_BIN_SIZE)
    angular_bins = np.arange(-ANGULAR_MAX_VEL, ANGULAR_MAX_VEL + ANGULAR_BIN_SIZE, ANGULAR_BIN_SIZE)
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(f"Mapa: {map_name}", fontsize=16, fontweight='bold')

    if raw is not None and VELOCITY_COL in raw.columns:
        axes[0].hist(raw[VELOCITY_COL].abs(), bins=linear_bins, alpha=0.5, 
                label=f"Przed (n={len(raw)})", color="orange", edgecolor="black")
    if proc is not None and VELOCITY_COL in proc.columns:
        axes[0].hist(proc[VELOCITY_COL].abs(), bins=linear_bins, alpha=0.7, 
                label=f"Po (n={len(proc)})", color="green", edgecolor="black")
    
    # Linia celu
    if proc is not None:
        axes[0].axhline(y=LINEAR_TARGET_PER_BIN, color='red', linestyle='--', 
                       label=f'Cel: {LINEAR_TARGET_PER_BIN}', linewidth=2)
    
    axes[0].set_title("Prędkość liniowa", fontsize=12, fontweight='bold')
    axes[0].set_xlabel("v [m/s]")
    axes[0].set_ylabel("Liczba próbek")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    if raw is not None and ANGULAR_COL in raw.columns:
        axes[1].hist(raw[ANGULAR_COL], bins=angular_bins, alpha=0.5, 
                label=f"Przed (n={len(raw)})", color="skyblue", edgecolor="black")
    if proc is not None and ANGULAR_COL in proc.columns:
        axes[1].hist(proc[ANGULAR_COL], bins=angular_bins, alpha=0.7, 
                label=f"Po (n={len(proc)})", color="steelblue", edgecolor="black")
    
    if proc is not None:
        axes[1].axhline(y=ANGULAR_TARGET_PER_BIN, color='red', linestyle='--',
                       label=f'Cel: {ANGULAR_TARGET_PER_BIN}', linewidth=2)
    
    axes[1].set_title("Prędkość kątowa", fontsize=12, fontweight='bold')
    axes[1].set_xlabel("ω [rad/s]")
    axes[1].set_ylabel("Liczba próbek")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def plot_combined_histogram(all_raw, all_proc):
    """Zbiorczy histogram dla wszystkich map"""
    linear_bins = np.arange(0, LINEAR_MAX_VEL + LINEAR_BIN_SIZE, LINEAR_BIN_SIZE)
    angular_bins = np.arange(-ANGULAR_MAX_VEL, ANGULAR_MAX_VEL + ANGULAR_BIN_SIZE, ANGULAR_BIN_SIZE)

    # Ujednolicenie kolumn przed łączeniem
    for df_list in [all_raw, all_proc]:
        if df_list:
            for df in df_list:
                if VELOCITY_COL not in df.columns:
                    df[VELOCITY_COL] = 0.0
                if ANGULAR_COL not in df.columns:
                    df[ANGULAR_COL] = 0.0

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Zbiorczy histogram - wszystkie mapy", fontsize=16, fontweight='bold')

    # Prędkość liniowa
    if all_raw:
        combined_raw = pd.concat(all_raw, ignore_index=True)
        axes[0].hist(combined_raw[VELOCITY_COL].abs(), bins=linear_bins, alpha=0.5,
                     label=f"Przed (n={len(combined_raw)})", color="orange", edgecolor="black")
    if all_proc:
        combined_proc = pd.concat(all_proc, ignore_index=True)
        axes[0].hist(combined_proc[VELOCITY_COL].abs(), bins=linear_bins, alpha=0.7,
                     label=f"Po (n={len(combined_proc)})", color="green", edgecolor="black")
        
        # Linia celu dla wszystkich map
        total_target = LINEAR_TARGET_PER_BIN * len(all_proc)
        axes[0].axhline(y=total_target, color='red', linestyle='--',
                       label=f'Cel: {total_target}', linewidth=2)

    axes[0].set_title("Prędkość liniowa", fontsize=12, fontweight='bold')
    axes[0].set_xlabel("v [m/s]")
    axes[0].set_ylabel("Liczba próbek")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # Prędkość kątowa
    if all_raw:
        combined_raw = pd.concat(all_raw, ignore_index=True)
        axes[1].hist(combined_raw[ANGULAR_COL], bins=angular_bins, alpha=0.5,
                     label=f"Przed (n={len(combined_raw)})", color="skyblue", edgecolor="black")
    if all_proc:
        combined_proc = pd.concat(all_proc, ignore_index=True)
        axes[1].hist(combined_proc[ANGULAR_COL], bins=angular_bins, alpha=0.7,
                     label=f"Po (n={len(combined_proc)})", color="steelblue", edgecolor="black")
        
        # Linia celu dla wszystkich map
        total_target = ANGULAR_TARGET_PER_BIN * len(all_proc)
        axes[1].axhline(y=total_target, color='red', linestyle='--',
                       label=f'Cel: {total_target}', linewidth=2)

    axes[1].set_title("Prędkość kątowa", fontsize=12, fontweight='bold')
    axes[1].set_xlabel("ω [rad/s]")
    axes[1].set_ylabel("Liczba próbek")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

def balance_linear_keeping_angular(df, target_per_bin=30000, bin_width=0.1):
    v = df["velocity_linear"].abs().to_numpy()
    bins = np.arange(0, LINEAR_MAX_VEL + bin_width, bin_width)
    df["vel_bin"] = np.digitize(v, bins) - 1
    
    balanced = []

    for b in sorted(df["vel_bin"].unique()):
        grp = df[df["vel_bin"] == b]
        if len(grp) == 0:
            continue
        
        if len(grp) < target_per_bin:
            grp_bal = grp.sample(target_per_bin, replace=True, random_state=42)
        else:
            grp_bal = grp.sample(target_per_bin, replace=False, random_state=42)

        balanced.append(grp_bal)

    out = pd.concat(balanced).sample(frac=1.0, random_state=42)
    out.drop(columns=["vel_bin"], inplace=True)
    return out

def plot_linear_for_angular_range(df, ang_min, ang_max, bins=30):
    subset = df[(df["velocity_angular"] >= ang_min) &
                (df["velocity_angular"] < ang_max)]

    if len(subset) == 0:
        print("Brak danych w tym przedziale.")
        return

    plt.figure(figsize=(8, 4))
    plt.hist(subset["velocity_linear"], bins=bins)
    plt.title(f"Rozkład prędkości liniowej dla prędkości kątowej {ang_min}–{ang_max}")
    plt.xlabel("Prędkość liniowa")
    plt.ylabel("Liczba próbek")
    plt.grid(alpha=0.3)
    plt.show()

def plot_angular_for_linear_range(df, lin_min, lin_max, bins=30):
    subset = df[(df["velocity_linear"] >= lin_min) &
                (df["velocity_linear"] < lin_max)]

    if len(subset) == 0:
        print("Brak danych w tym przedziale.")
        return

    plt.figure(figsize=(8, 4))
    plt.hist(subset["velocity_angular"], bins=bins)
    plt.title(f"Rozkład prędkości kątowej dla prędkości liniowej {lin_min}–{lin_max}")
    plt.xlabel("Prędkość kątowa")
    plt.ylabel("Liczba próbek")
    plt.grid(alpha=0.3)
    plt.show()

def main():
    option = "laser_encoder"
    folders = [
        # f"csv_output/willowgarage/{option}",
        # f"csv_output/maze_1/{option}",
        f"csv_output/maze_3/{option}",      
    ]
    
    num_maps = len(folders)

    # KROK 1: Analiza rozkładu
    print("="*70)
    print("KROK 1: ANALIZA ROZKŁADU DANYCH")
    print("="*70)
    
    linear_stats, angular_stats = analyze_distribution(folders, option)
    print_distribution_analysis(linear_stats, angular_stats, num_maps)
    
    # KROK 2: Określ biny priorytetowe
    priority_linear = get_priority_bins(linear_stats, LINEAR_TARGET_PER_BIN, num_maps) if AUTO_BALANCE else None
    priority_angular = get_priority_bins(angular_stats, ANGULAR_TARGET_PER_BIN, num_maps) if AUTO_BALANCE else None
    
    if AUTO_BALANCE:
        print("\n" + "="*70)
        print("BINY PRIORYTETOWE (zwiększony limit)")
        print("="*70)
        if priority_linear:
            print(f"Linear: {sorted(priority_linear)}")
        if priority_angular:
            print(f"Angular: {sorted(priority_angular)}")

    # KROK 3: Przetwarzanie
    print("\n" + "="*70)
    print("KROK 2: PRZETWARZANIE MAP")
    print("="*70)

    processed_all = []
    
    results = []
    for folder in folders:
        map_name = os.path.basename(os.path.dirname(os.path.normpath(folder)))
        print(f"\nPrzetwarzanie mapy: {map_name}")
        raw, proc = process_map(folder, priority_linear=priority_linear, priority_angular=priority_angular)
        results.append((map_name, raw, proc))

        processed_all.append(proc)

    df_all = pd.concat(processed_all, ignore_index=True)
    print("Połączono dane:", df_all.shape)

    # plot_linear_for_angular_range(df_all, -2.0, -1.0)
    plot_linear_for_angular_range(df_all, 1.5, 2.0)
    # plot_linear_for_angular_range(df_all, -0.1, 0.1)
    # plot_angular_for_linear_range(df_all, 0.8, 1.2)
    # plot_angular_for_linear_range(df_all, 0.0, 0.1)

    df_balanced = balance_linear_keeping_angular(
        df_all,
        target_per_bin=30000
    )
    if not os.path.isfile("processed_balanced/final_balanced.csv"):
        df_balanced.to_csv("processed_balanced/final_balanced.csv", index=False)

    # KROK 4: Wizualizacja dla poszczególnych map
    print("\n" + "="*70)
    print("KROK 3: WIZUALIZACJA POSZCZEGÓLNYCH MAP")
    print("="*70)
    
    all_raw = []
    all_proc = []
    
    for map_name, raw, proc in results:
        if raw is not None or proc is not None:
            plot_histogram(raw, proc, map_name)
        if raw is not None:
            all_raw.append(raw)
        if proc is not None:
            all_proc.append(proc)

    # KROK 5: Zbiorczy histogram
    print("\n" + "="*70)
    print("KROK 4: ZBIORCZY HISTOGRAM DLA WSZYSTKICH MAP")
    print("="*70)
    
    if all_raw and all_proc:
        plot_combined_histogram(all_raw, all_proc)

    plot_combined_histogram([df_balanced], [])

    # Podsumowanie
    print("\n" + "="*70)
    print("PODSUMOWANIE KOŃCOWE")
    print("="*70)
    
    total_linear_bins = int(LINEAR_MAX_VEL / LINEAR_BIN_SIZE) + 1
    total_angular_bins = int(ANGULAR_MAX_VEL / ANGULAR_BIN_SIZE) + 1
    
    print(f"\nCele:")
    print(f"  Linear: {LINEAR_TARGET_PER_BIN} próbek/bin × {num_maps} map × {total_linear_bins} binów = {LINEAR_TARGET_PER_BIN * num_maps * total_linear_bins} próbek")
    print(f"  Angular: {ANGULAR_TARGET_PER_BIN} próbek/bin × {num_maps} map × {total_angular_bins} binów = {ANGULAR_TARGET_PER_BIN * num_maps * total_angular_bins} próbek")
    
    if results:
        for map_name, _, proc in results:
            if proc is not None:
                print(f"\n{map_name}: {len(proc)} rekordów")


if __name__ == "__main__":
    main()
