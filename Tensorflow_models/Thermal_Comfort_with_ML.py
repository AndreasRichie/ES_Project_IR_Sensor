# %%
import numpy as np
import pandas as pd
import tensorflow as tf
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.gridspec as gridspec
import os
import sys
import ydf
import typing
import sklearn.metrics
from Two_Nodes_Custom import two_nodes_gagge_ml_skin_temp
from pythermalcomfort.utilities import mean_radiant_tmp, body_surface_area
from pythermalcomfort.classes_return import GaggeTwoNodes
from pythermalcomfort.models import two_nodes_gagge
from scipy.stats import pearsonr

# Make NumPy printouts easier to read.
np.set_printoptions(precision=3, suppress=True)

image_path = os.path.abspath("f:\\Dokumente\\FH\\Masterarbeit\\Master_Thesis_Reichenauer\\img")

use_ydf = False

# load model
path_to_models = os.path.join(os.getcwd(), "Saved_Models")
if use_ydf:
    model_load = os.path.join(path_to_models, "RFR_Model.keras")
else:
    model_load = os.path.join(path_to_models, "ANNR_Model.keras")

skin_temp_model = None

if os.path.exists(model_load):
    if use_ydf:
        skin_temp_model = ydf.load_model(model_load)
    else:
        if os.path.isfile(model_load):
            skin_temp_model = tf.keras.models.load_model(model_load)

if skin_temp_model == None:
    print("No model could be loaded!")
    sys.exit(-1)

# load ASHRAE RP 844 dataset
path_to_datasets = os.path.join(os.getcwd(), "Datasets")
dataset_file = os.path.join(path_to_datasets, "ASHRAE_RP_884.csv")

# column names
columns = ["ta", "tr", "tg", "top", "rh", "vel", "clo", "activity_60", "activity_30",
           "ht", "wt", "thermal_sensation", "pmv", "thermal_comfort", "ppd"]

# read file and load dataset
dataset = pd.read_csv(dataset_file, usecols=columns, low_memory=False)

# merge activity_60 and activity_30
dataset["met"] = dataset["activity_60"].fillna(dataset["activity_30"])
# fill rest with standard value of 1.1 met
dataset["met"] = dataset["met"].fillna(1.1)

# convert W/m^2 to met, every value over 10 can safely be assumed to not be in met
dataset["met"] = np.where(
    dataset["met"] > 10, dataset["met"] / 58.15, dataset["met"])

# remove all with met < 1.0 or met > 1.3
dataset = dataset[(dataset["met"] >= 1.0) & (dataset["met"] <= 1.3)]

# remove all with clo < 0.3 or clo > 1.2, includes missing
dataset = dataset[(dataset["clo"] >= 0.3) & (dataset["clo"] <= 1.2)]

# remove all with rh or vel missing
dataset.dropna(subset=["rh", "vel"], inplace=True)


# convert tg to tr wherever tr is missing
def convert_globe_temp_to_mrt(dataframe: pd.DataFrame, globe_diameter: float = 0.15) -> pd.DataFrame:
    # get all values where tr is missing but tg is available
    mask = dataframe["tr"].isna() & dataframe["tg"].notna()

    # calculate tr for the masked rows with standard emissivity of black globe
    derived_tr = dataframe[mask].apply(
        lambda row: mean_radiant_tmp(
            tg=row["tg"],
            tdb=row["ta"],
            v=row["vel"],
            d=globe_diameter,
            emissivity=0.95
        ),
        axis=1
    )

    # inject calculated tr values to dataframe
    dataframe.loc[mask, "tr"] = derived_tr

    return dataframe


dataset = convert_globe_temp_to_mrt(dataset)


# convert top to tr wherever tr is missing
def convert_operative_temp_to_mrt(dataframe: pd.DataFrame) -> pd.DataFrame:

    # get all values where tr is missing but top and ta are available
    mask = dataframe["tr"].isna(
    ) & dataframe["top"].notna() & dataframe["ta"].notna()

    # extract series for vectorization
    top = dataframe.loc[mask, "top"]
    ta = dataframe.loc[mask, "ta"]
    vel = dataframe.loc[mask, "vel"]

    # define ASHRAE 55 air speed thresholds and weights (A)
    conditions = [
        vel < 0.2,
        (vel >= 0.2) & (vel < 0.6),
        vel >= 0.6
    ]
    choices = [0.5, 0.6, 0.7]

    # apply conditions to create an array of weights
    A = np.select(conditions, choices, default=0.5)

    # use reversed equation of operative temperature for calculation of MRT
    derived_tr = (top - A * ta) / (1.0 - A)

    # Inject into dataframe
    dataframe.loc[mask, "tr"] = derived_tr

    return dataframe


dataset = convert_globe_temp_to_mrt(dataset)

# fill missing ta with top
mask = dataset["ta"].isna() & dataset["top"].notna()
dataset.loc[mask, "ta"] = dataset.loc[mask, "top"]

# fill missing tr with top
mask = dataset["tr"].isna() & dataset["top"].notna()
dataset.loc[mask, "tr"] = dataset.loc[mask, "top"]

# fill missing tr with ta
mask = dataset["tr"].isna() & dataset["ta"].notna()
dataset.loc[mask, "tr"] = dataset.loc[mask, "ta"]

# remove all with ta or tr missing
dataset.dropna(subset=["ta", "tr"], inplace=True)

# remove all where relevant outputs are missing
dataset.dropna(subset=["thermal_sensation", "pmv"], inplace=True)


# add skin area column and fill with value where wt and ht exists
def add_skin_area(dataframe: pd.DataFrame) -> pd.DataFrame:
    mask = dataframe["wt"].notna() & dataframe["ht"].notna()

    # calculate body surface for wt and ht values
    body_area = dataframe[mask].apply(
        lambda row: body_surface_area(
            weight=row["wt"],
            height=row["ht"]
        ),
        axis=1
    )

    dataframe["bsa"] = body_area

    # fill missing with pythermalcomfort two-node default of 1.8258
    dataframe["bsa"] = dataframe["bsa"].fillna(1.8258)

    return dataframe


dataset = add_skin_area(dataset)

# add skin temperature column and fill with model outputs


def add_skin_temperature(dataframe: pd.DataFrame, model: tf.keras.Model) -> pd.DataFrame:
    if use_ydf:
        env_values = dataframe[["ta", "rh", "tr"]].copy()
        rename_map = {
            "ta": "air_temp",
            "tr": "mrt",
            "rh": "air_hum"
        }
        env_values = typing.cast(pd.DataFrame, env_values)
        env_values = env_values.rename(columns=rename_map)
    else:
        env_values = dataframe[["ta", "rh", "tr"]].to_numpy(dtype="float32")

    t_skin_pred = model.predict(env_values)

    dataframe["tsk_ml"] = t_skin_pred.flatten()

    return dataframe


dataset = add_skin_temperature(dataset, skin_temp_model)


# remove rows with tsk over 36°C and under 28°C, since these are physiological limits for healthy humans
dataset = dataset[(dataset["tsk_ml"] >= 28.0) & (dataset["tsk_ml"] <= 36.0)]

# calculate modified two nodes model outputs for data
two_node_mod_results = two_nodes_gagge_ml_skin_temp(
    tdb=dataset["ta"].to_numpy(),
    tr=dataset["tr"].to_numpy(),
    v=dataset["vel"].to_numpy(),
    rh=dataset["rh"].to_numpy(),
    met=dataset["met"].to_numpy(),
    clo=dataset["clo"].to_numpy(),
    tskml=dataset["tsk_ml"].to_numpy(),
    body_surface_area=dataset["bsa"].to_numpy(),
)

if isinstance(two_node_mod_results, GaggeTwoNodes):
    dataset["pmv_two_node_mod"] = two_node_mod_results.pmv_set
    dataset["therm_sens_two_node_mod"] = two_node_mod_results.t_sens
    dataset["therm_comf_two_node_mod"] = two_node_mod_results.disc

dataset = typing.cast(pd.DataFrame, dataset)

# calculate original two nodes model outputs for comparison
two_node_orig_results = two_nodes_gagge(
    tdb=dataset["ta"].to_numpy(),
    tr=dataset["tr"].to_numpy(),
    v=dataset["vel"].to_numpy(),
    rh=dataset["rh"].to_numpy(),
    met=dataset["met"].to_numpy(),
    clo=dataset["clo"].to_numpy(),
    body_surface_area=dataset["bsa"].to_numpy(),
)

if isinstance(two_node_orig_results, GaggeTwoNodes):
    dataset["pmv_two_node_orig"] = two_node_orig_results.pmv_set
    dataset["therm_sens_two_node_orig"] = two_node_orig_results.t_sens
    dataset["therm_comf_two_node_orig"] = two_node_orig_results.disc
    dataset["tsk_gagge"] = two_node_orig_results.t_skin

#%%
def plot_tsk_dual_analysis(y_true: np.ndarray, y_pred: np.ndarray) -> None:
    """!
    @brief Generates dual-panel analysis plot for skin temperature validation.
    @details Creates 1x2 figure. Left = Scatter (Correlation). Right = Bland-Altman (Agreement).
    Calculates and embeds r, R^2, MAE, RMSE, and MBE in respective panels.
    @param y_true Numpy array of ground truth values (Original Two-Node).
    @param y_pred Numpy array of predicted values (ML Modified Two-Node).
    @return None
    """
    
    # 1. Error Metrics (Agreement)
    differences = y_pred - y_true
    means = (y_pred + y_true) / 2.0
    
    mae = sklearn.metrics.mean_absolute_error(y_true, y_pred)
    rmse = sklearn.metrics.root_mean_squared_error(y_true, y_pred)
    mbe = np.mean(differences)
    std_diff = np.std(differences)
    
    upper_loa = mbe + 1.96 * std_diff
    lower_loa = mbe - 1.96 * std_diff

    # 2. Correlation Metrics (Trend)
    r_val, _ = pearsonr(y_true, y_pred)
    r2_val = sklearn.metrics.r2_score(y_true, y_pred)

    # 3. Setup Figure
    fig, axes = plt.subplots(1, 2, figsize=(15, 7))
    ax1, ax2 = axes[0], axes[1]
    props = dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray')
    
    # --- PANEL A: SCATTER PLOT ---
    ax1.scatter(y_true, y_pred, alpha=0.3, color='steelblue', edgecolor='k')
    
    min_val = min(np.min(y_true), np.min(y_pred)) - 0.5
    max_val = max(np.max(y_true), np.max(y_pred)) + 0.5
    ax1.plot([min_val, max_val], [min_val, max_val], 'r--', linewidth=2, label='Perfect Tracking')
    
    # Text box for Correlation
    textstr_corr = '\n'.join((
        r'$r=%.2f$' % (r_val, ),
        r'$R^2=%.2f$' % (r2_val, )))
    
    ax1.text(0.05, 0.95, textstr_corr, transform=ax1.transAxes, fontsize=14,
             verticalalignment='top', bbox=props)
    
    ax1.set_title('Scatter Plot for Absolute Trend Tracking', fontsize=16, fontweight='bold')
    ax1.set_xlabel(r'Original Two-Node Model Skin Temperature $T_{sk,TN}$ in °C', fontsize=14)
    ax1.set_ylabel(r'ML Model Skin Temperature $T_{sk,ML}$ in °C', fontsize=14)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='lower right', fontsize=13)
    
    # --- PANEL B: BLAND-ALTMAN ---
    ax2.scatter(means, differences, alpha=0.3, color='forestgreen', edgecolor='k')
    
    ax2.axhline(mbe, color='red', linestyle='-', linewidth=2, label='MBE')
    ax2.axhline(upper_loa, color='black', linestyle='--', linewidth=1.5, label='±1.96 SD')
    ax2.axhline(lower_loa, color='black', linestyle='--', linewidth=1.5)
    ax2.axhline(0, color='gray', linestyle=':', linewidth=1)
    
    # Text box for Error
    textstr_err = '\n'.join((
        r'$\mathrm{MAE}=%.2f$ °C' % (mae, ),
        r'$\mathrm{RMSE}=%.2f$ °C' % (rmse, ),
        r'$\mathrm{MBE}=%.2f$ °C' % (mbe, )))
    
    ax2.text(0.05, 0.95, textstr_err, transform=ax2.transAxes, fontsize=14,
             verticalalignment='top', bbox=props)
    
    ax2.set_title('Bland-Altman Plot for Error Variance', fontsize=16, fontweight='bold')
    ax2.set_xlabel(r'Mean $T_{sk}$ in °C', fontsize=14)
    ax2.set_ylabel('Error $T_{sk,ML}$-$T_{sk,TN}$ in °C', fontsize=14)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=13)
    
    # 4. Formatting & Save
    ax1.tick_params(axis='both', labelsize=14)
    ax2.tick_params(axis='both', labelsize=14)
    ax1.tick_params(axis='y', labelsize=14)
    ax2.tick_params(axis='y', labelsize=14)

    plt.tight_layout()
    if(os.path.exists(image_path)):
        plt.savefig(os.path.join(image_path,"Skin_Temp_Comparison.png"), dpi=200, bbox_inches='tight')
    plt.show()

# Execute (Ensure NaNs are dropped prior to execution)
dataset = dataset.dropna(subset=['tsk_gagge', 'tsk_ml'])
plot_tsk_dual_analysis(dataset['tsk_gagge'].to_numpy(), dataset['tsk_ml'].to_numpy())


#%%
def plot_error_vs_features(df: pd.DataFrame, tsk_true_col: str, tsk_pred_col: str) -> None:
    # 1. Calculate Error
    df = df.dropna(subset=[tsk_true_col, tsk_pred_col]).copy()
    df['error'] = df[tsk_pred_col] - df[tsk_true_col]
    
    # 2. Define Features to Plot (Map column names to display labels)
    # UPDATE these keys to match your exact dataframe column names
    features = {
        'ta': 'Air Temperature ($T_a$) in °C',
        'rh': 'Relative Humidity ($RH_a$) in %',
        'tr': 'Mean Radiant Temp ($T_{MRT}$) in °C',
        'vel': 'Air Velocity ($v_a$) in m/s',
        'clo': 'Clothing Value ($k_{Cl}$) in clo',
        'met': 'Activity Level ($k_{Al}$) in met'
    }

    slope_units = {
        'ta': '°C/°C',
        'rh': '°C/%',
        'tr': '°C/°C',
        'vel': '°C/(m/s)',
        'clo': '°C/clo',
        'met': '°C/met'
    }
    
    # 3. Setup Grid
    fig, axes = plt.subplots(2, 3, figsize=(18, 10), sharey=True)
    axes = axes.flatten()
    
    # 4. Generate Subplots
    for i, (col, label) in enumerate(features.items()):
        ax = axes[i]
        
        # Check if column exists to prevent crashes
        if col not in df.columns:
            ax.text(0.5, 0.5, f"Column '{col}' not found", ha='center')
            continue
            
        x = df[col].to_numpy()
        y = df['error'].to_numpy()
        
        # Scatter data
        ax.scatter(x, y, alpha=0.3, color='steelblue', edgecolor='k', s=20)
        
        # Zero error reference line
        ax.axhline(0, color='gray', linestyle=':', linewidth=1.5)
        
        # Calculate and plot trendline (1st degree polynomial)
        try:
            z = np.polyfit(x, y, 1)
            p = np.poly1d(z)
            x_trend = np.linspace(x.min(), x.max(), 100)
            ax.plot(x_trend, p(x_trend), "r--", linewidth=2, label=f"Trend (Slope: {z[0]:.2f}{slope_units[col]})")
            ax.legend(loc='upper right', fontsize=13)
        except Exception:
            pass # Skip trendline if data lacks variance (e.g., all clo values are identical)
            
        # Formatting
        ax.set_xlabel(label, fontsize=14, fontweight='bold')
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.tick_params(axis='both', labelsize=14)
        
        # Only add Y-label to the leftmost plots to reduce clutter
        if i % 3 == 0:
            ax.set_ylabel('Error $T_{sk,ML}$-$T_{sk,TN}$ in °C', fontsize=14)
            ax.tick_params(axis='y', labelsize=14)

    plt.suptitle(r'Systematic Error Analysis: $\Delta T_{sk}$ vs. Input Features', fontsize=18, fontweight='bold', y=1)
    plt.tight_layout()
    if(os.path.exists(image_path)):
        plt.savefig(os.path.join(image_path,"Skin_Temp_Diff_Plots.png"), dpi=200, bbox_inches='tight')
    plt.show()

# Execute 
plot_error_vs_features(dataset, tsk_true_col='tsk_gagge', tsk_pred_col='tsk_ml')
#%%

def get_error_metrics(df: pd.DataFrame, truth_col: str, pred_col: str)-> list[float]:
    # TSV values are used as true values for comparison
    y_true = df[truth_col]
    mae = sklearn.metrics.mean_absolute_error(y_true, df[pred_col])
    rmse = sklearn.metrics.root_mean_squared_error(y_true, df[pred_col])
    mbe = float(np.mean(df[pred_col] - y_true))
    return [mae, rmse, mbe]

def print_error_metrics(header: str, baseline: float,orig_t_sens: float, orig_pmv: float, mod_t_sens: float, mod_pmv: float)->None:
    print(header)
    print(f"Baseline (Standard PMV): {baseline:.3f}")
    print(f"Original Two-Node Model (Two-Node t_sens): {orig_t_sens:.3f}")
    print(f"Original Two-Node Model (Two-Node pmv_set): {orig_pmv:.3f}")
    print(f"Modified Two-Node Model (Two-Node t_sens): {mod_t_sens:.3f}")
    print(f"Modified Two-Node Model (Two-Node pmv_set): {mod_pmv:.3f}\n")

def print_improvement(error: str, type: str, baseline:float, compare:float) -> None:
    direction_string =""
    if(error=="MBE"):
        direction_mbe = np.sign(compare)/np.sign(baseline) < 0
        direction_string = f", Direction change: {direction_mbe}"
        
    improvement = ((abs(baseline) - abs(compare)) / abs(baseline)) * 100
    print(f"Improvement {error} {type} vs Baseline: {improvement:.1f}%{direction_string}")

def plot_error_metrics_comparison(baseline_scores: list[float],orig_pred_ts_scores: list[float],orig_pmv_set_scores: list[float], mod_pred_ts_scores: list[float],mod_pmv_set_scores: list[float])-> None:
    # 1. Define Categories and Data
    metrics = ['MAE', 'RMSE', 'MBE']
    
    # 2. Set up bar positioning
    x = np.arange(len(metrics))  # [0, 1, 2]
    width = 0.15                 # Width of each bar
    
    # 3. Create Figure
    fig, ax = plt.subplots(figsize=(12, 6))
    
    # Plot each model's bars, offset by the width
    rects1 = ax.bar(x - 2*width, baseline_scores, width, label='Baseline A (PMV RP-884)', color='firebrick', alpha=0.8)
    rects2 = ax.bar(x - width, orig_pred_ts_scores, width, label='Predicted TS (Original Two-Node)', color='cornflowerblue', alpha=0.9)
    rects3 = ax.bar(x, mod_pred_ts_scores, width, label='Predicted TS (Modified Two-Node)', color='steelblue', alpha=0.9)
    rects4 = ax.bar(x + width, orig_pmv_set_scores, width, label='PMV SET (Original Two-Node)', color='mediumseagreen', alpha=0.9)
    rects5 = ax.bar(x + 2*width, mod_pmv_set_scores, width, label='PMV SET (Modified Two-Node)', color='forestgreen', alpha=0.9)
    
    # 4. Formatting and Labels
    ax.set_ylabel('Error Margin in Scale Units', fontsize=14)
    ax.set_title('Modified Two-Node Model Performance Comparison: Error Metrics', fontsize=16, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(metrics, fontsize=14)
    ax.legend(fontsize=13)
    
    # Add a solid line at y=0 to anchor the negative MBE values
    ax.axhline(0, color='black', linewidth=1.2)
    
    # Optional: Add gridlines for easier reading
    ax.grid(axis='y', linestyle='--', alpha=0.6)
    ax.tick_params(axis='y', labelsize=14)
    
    plt.tight_layout()
    
    # dpi=300 is publication quality. 
    # bbox_inches='tight' prevents labels from being cut off in the saved file.
    if(os.path.exists(image_path)):
        plt.savefig(os.path.join(image_path,"Error_Metrics_Comparison.png"), dpi=300, bbox_inches='tight')

    plt.tight_layout()
    plt.show()


def evaluate_improvement(dataframe: pd.DataFrame):

    # ensure no NaN values in the target comparison columns
    eval_df = dataframe.dropna(
        subset=['thermal_sensation', 'pmv', 'therm_sens_two_node_mod', 'pmv_two_node_mod', "therm_sens_two_node_orig", "pmv_two_node_orig"])


    # Baseline: Standard PMV loaded from RP884 set
    errors_pmv = get_error_metrics(eval_df, "thermal_sensation", "pmv")

    # Original Two-Node Model: Predicted Thermal Sensation (t_sens)
    errors_t_sens_orig = get_error_metrics(eval_df, "thermal_sensation", "therm_sens_two_node_orig")

    # Original Two-Node Model: PMV calculated via SET (pmv_set)
    errors_pmv_set_orig = get_error_metrics(eval_df, "thermal_sensation", "pmv_two_node_orig")

    # Modified Two-Node Model: Predicted Thermal Sensation (t_sens)
    errors_t_sens_mod = get_error_metrics(eval_df, "thermal_sensation", "therm_sens_two_node_mod")

    # Modified Two-Node Model: PMV calculated via SET (pmv_set)
    errors_pmv_set_mod = get_error_metrics(eval_df, "thermal_sensation", "pmv_two_node_mod")

    print_error_metrics("--- Mean Absolute Error (Lower is Better) ---", errors_pmv[0],errors_t_sens_orig[0],errors_pmv_set_orig[0],errors_t_sens_mod[0],errors_pmv_set_mod[0])
    print_error_metrics("--- Root Mean Squared Error (Lower is Better) ---", errors_pmv[1],errors_t_sens_orig[1],errors_pmv_set_orig[1],errors_t_sens_mod[1],errors_pmv_set_mod[1])
    print_error_metrics("--- Mean Bias Error (Lower is Better) ---", errors_pmv[2],errors_t_sens_orig[2],errors_pmv_set_orig[2],errors_t_sens_mod[2],errors_pmv_set_mod[2])

    # Calculate improvement percentage
    print_improvement("MAE","Thermal Sensation Original", errors_pmv[0], errors_t_sens_orig[0])
    print_improvement("MAE","PMV SET Original", errors_pmv[0], errors_pmv_set_orig[0])
    print_improvement("MAE","Thermal Sensation Modified", errors_pmv[0], errors_t_sens_mod[0])
    print_improvement("MAE","PMV SET Modified", errors_pmv[0], errors_pmv_set_mod[0])

    print_improvement("RMSE","Thermal Sensation Original", errors_pmv[1], errors_t_sens_orig[1])
    print_improvement("RMSE","PMV SET Original", errors_pmv[1], errors_pmv_set_orig[1])
    print_improvement("RMSE","Thermal Sensation Modified", errors_pmv[1], errors_t_sens_mod[1])
    print_improvement("RMSE","PMV SET Modified", errors_pmv[1], errors_pmv_set_mod[1])

    print_improvement("MBE","Thermal Sensation Original", errors_pmv[2], errors_t_sens_orig[2])
    print_improvement("MBE","PMV SET Original", errors_pmv[2], errors_pmv_set_orig[2])
    print_improvement("MBE","Thermal Sensation Modified", errors_pmv[2], errors_t_sens_mod[2])
    print_improvement("MBE","PMV SET Modified", errors_pmv[2], errors_pmv_set_mod[2])

    plot_error_metrics_comparison(errors_pmv,errors_t_sens_orig,errors_pmv_set_orig, errors_t_sens_mod, errors_pmv_set_mod)

# Execute
evaluate_improvement(dataset)

#%%

def get_accuracy_metrics(df: pd.DataFrame, truth_col: str, pred_col: str)-> list[float]:
    # TSV values are used as true values for comparison
    y_true = df[truth_col].astype(int).to_numpy()

    # Round and FORCE integer cast
    y_pred_rounded = round(df[pred_col]).round().astype(int).to_numpy()

    # 1. Exact Match Accuracy
    exact_accuracy = float(sklearn.metrics.accuracy_score(y_true, y_pred_rounded)) * 100

    # 2. Accuracy within ±1 vote
    # Calculate absolute difference between truth and rounded prediction
    abs_error = np.abs(y_true - y_pred_rounded)
    # Count how many predictions have an error of 1 or 0
    within_one_accuracy = np.mean(abs_error <= 1)  * 100

    return [exact_accuracy, within_one_accuracy]

def print_accuracy_metrics(header: str, exact_accuracy: float, within_one_accuracy: float)->None:
    print(f"--- Accuracy Metrics: {header} ---")
    print(f"Exact Match: {exact_accuracy:.1f}%")
    print(f"Match ± 1 Unit: {within_one_accuracy:.1f}%\n")


def plot_accuracy_metrics_comparison(baseline_scores: list[float],orig_pred_ts_scores: list[float],orig_pmv_set_scores: list[float], mod_pred_ts_scores: list[float],mod_pmv_set_scores: list[float])-> None:
    # 1. Define Categories and Data
    metrics = ['Exact Match', '± 1 Unit Match']
    
    # 2. Set up bar positioning
    x = np.arange(len(metrics))  # [0, 1, 2]
    width = 0.15                 # Width of each bar
    
    # 3. Create Figure
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Plot each model's bars, offset by the width
    rects1 = ax.bar(x - 2*width, baseline_scores, width, label='Baseline A (PMV RP-884)', color='firebrick', alpha=0.8)
    rects2 = ax.bar(x - width, orig_pred_ts_scores, width, label='Predicted TS (Original Two-Node)', color='cornflowerblue', alpha=0.9)
    rects3 = ax.bar(x, mod_pred_ts_scores, width, label='Predicted TS (Modified Two-Node)', color='steelblue', alpha=0.9)
    rects4 = ax.bar(x + width, orig_pmv_set_scores, width, label='PMV SET (Original Two-Node)', color='mediumseagreen', alpha=0.9)
    rects5 = ax.bar(x + 2*width, mod_pmv_set_scores, width, label='PMV SET (Modified Two-Node)', color='forestgreen', alpha=0.9)
    
    # 4. Formatting and Labels
    ax.set_ylabel('Accuracy in %', fontsize=14)
    ax.set_title('Modified Two-Node Model Performance Comparison: Accuracy Metrics', fontsize=16, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(metrics, fontsize=14)
    ax.legend(fontsize=13)
    
    # Add a solid line at y=0 to anchor the negative MBE values
    ax.axhline(0, color='black', linewidth=1.2)
    
    # Optional: Add gridlines for easier reading
    ax.grid(axis='y', linestyle='--', alpha=0.6)
    ax.tick_params(axis='y', labelsize=14)
    
    plt.tight_layout()
    
    # dpi=300 is publication quality. 
    # bbox_inches='tight' prevents labels from being cut off in the saved file.
    if(os.path.exists(image_path)):
        plt.savefig(os.path.join(image_path,"Accuracy_Metrics_Comparison.png"), dpi=300, bbox_inches='tight')

    plt.show()


def calculate_accuracy_metrics(df: pd.DataFrame) -> None:

    # ensure no NaN values in the target comparison columns
    eval_df = df.dropna(
        subset=['thermal_sensation', 'pmv', 'therm_sens_two_node_mod', 'pmv_two_node_mod', "therm_sens_two_node_orig", "pmv_two_node_orig"])

    accuracy_pmv = get_accuracy_metrics(eval_df, truth_col='thermal_sensation', pred_col='pmv')
    accuracy_t_sens_orig = get_accuracy_metrics(eval_df, truth_col='thermal_sensation', pred_col='therm_sens_two_node_orig')
    accuracy_pmv_set_orig = get_accuracy_metrics(eval_df, truth_col='thermal_sensation', pred_col='pmv_two_node_orig')
    accuracy_t_sens_mod = get_accuracy_metrics(eval_df, truth_col='thermal_sensation', pred_col='therm_sens_two_node_mod')
    accuracy_pmv_set_mod = get_accuracy_metrics(eval_df, truth_col='thermal_sensation', pred_col='pmv_two_node_mod')

    print_accuracy_metrics("Standard PMV", accuracy_pmv[0], accuracy_pmv[1])
    print_accuracy_metrics("Thermal Sensation Original", accuracy_t_sens_orig[0], accuracy_t_sens_orig[1])
    print_accuracy_metrics("PMV SET Original", accuracy_pmv_set_orig[0], accuracy_pmv_set_orig[1])
    print_accuracy_metrics("Thermal Sensation Modified", accuracy_t_sens_mod[0], accuracy_t_sens_mod[1])
    print_accuracy_metrics("PMV SET Modified", accuracy_pmv_set_mod[0], accuracy_pmv_set_mod[1])

    plot_accuracy_metrics_comparison(accuracy_pmv,accuracy_t_sens_orig,accuracy_pmv_set_orig, accuracy_t_sens_mod, accuracy_pmv_set_mod)

# Execute
calculate_accuracy_metrics(dataset)


#%%
def evaluate_comfort_acceptability(dataframe: pd.DataFrame) -> pd.DataFrame:

    eval_df = dataframe.dropna(
        subset=['thermal_comfort', 'therm_comf_two_node_mod', "pmv", "therm_comf_two_node_orig"])
    # 1. Create binary targets (1 = Acceptable, 0 = Unacceptable)
    # RP-884: 4,5,6 are 'Acceptable'
    eval_df['comfort_binary'] = (eval_df['thermal_comfort'] >= 4).astype(int)

    # DISC: Range (>-0.5 to <0.5) is 'Acceptable'
    eval_df['disc_binary_orig'] = (
        eval_df['therm_comf_two_node_orig'].abs() < 0.5).astype(int)
    eval_df['disc_binary_mod'] = (
        eval_df['therm_comf_two_node_mod'].abs() < 0.5).astype(int)

    # PMV: Range (-0.5 to 0.5) is 'Acceptable'
    eval_df['pmv_comf_binary'] = (
        eval_df['pmv'].abs() <= 0.5).astype(int)

    # 2. Compare using classification report
    print("--- Acceptability Classification Report PMV ---")
    print(sklearn.metrics.classification_report(
        eval_df['comfort_binary'], eval_df['pmv_comf_binary']))

    print("--- Acceptability Classification Report DISC Original ---")
    print(sklearn.metrics.classification_report(
        eval_df['comfort_binary'], eval_df['disc_binary_orig']))

    print("--- Acceptability Classification Report DISC Modified ---")
    print(sklearn.metrics.classification_report(
        eval_df['comfort_binary'], eval_df['disc_binary_mod']))

    # 3. View confusion matrix to identify bias
    # Bias: Does DISC predict 'Unacceptable' too often (over-sensitive)?
    print("\n--- Confusion Matrix PMV ---")
    print(sklearn.metrics.confusion_matrix(
        eval_df['comfort_binary'], eval_df['pmv_comf_binary']))

    print("--- Confusion Matrix DISC Original ---")
    print(sklearn.metrics.confusion_matrix(
        eval_df['comfort_binary'], eval_df['disc_binary_orig']))

    print("--- Confusion Matrix DISC Modified ---")
    print(sklearn.metrics.confusion_matrix(
        eval_df['comfort_binary'], eval_df['disc_binary_mod']))
    
    return eval_df
    

# Execute
dataset = evaluate_comfort_acceptability(dataset)

def plot_all_matrices(df, truth_col, pmv_col, orig_col, mod_col):
    """
    @brief Generates a 1x3 figure containing confusion matrices for three models.
    @param df Pandas DataFrame containing classification results.
    @param truth_col String name of ground truth column.
    @param pmv_col String name of PMV prediction column.
    @param orig_col String name of Original Two-Node prediction column.
    @param mod_col String name of ML Modified Two-Node prediction column.
    @return None
    """
    df_clean = df.dropna(subset=[truth_col, pmv_col, orig_col, mod_col])
    y_true = df_clean[truth_col]
    
    cm_pmv = sklearn.metrics.confusion_matrix(y_true, df_clean[pmv_col])
    cm_orig = sklearn.metrics.confusion_matrix(y_true, df_clean[orig_col])
    cm_mod = sklearn.metrics.confusion_matrix(y_true, df_clean[mod_col])
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 6))

    labels = ["Unacceptable", "Acceptable"]
    
    sns.heatmap(cm_pmv, annot=True, fmt='d', cmap='Reds', ax=axes[0], cbar=False)
    axes[0].set_title('Baseline A (PMV RP-884)', fontsize=16)
    axes[0].set_xticklabels(labels, fontsize=14)
    axes[0].set_yticklabels(labels, fontsize=14)
    
    sns.heatmap(cm_orig, annot=True, fmt='d', cmap='Blues', ax=axes[1], cbar=False)
    axes[1].set_title('Original Two-Node (DISC)', fontsize=16)
    axes[1].set_xticklabels(labels, fontsize=14)
    axes[1].set_yticklabels(labels, fontsize=14)
    
    sns.heatmap(cm_mod, annot=True, fmt='d', cmap='Greens', ax=axes[2], cbar=False)
    axes[2].set_title('Modified Two-Node (DISC)', fontsize=16)
    axes[2].set_xticklabels(labels, fontsize=14)
    axes[2].set_yticklabels(labels, fontsize=14)
    
    for ax in axes:
        ax.set_ylabel('Actual Comfort', fontsize=14)
        ax.set_xlabel('Predicted Comfort', fontsize=14)

    fig.suptitle("Confusion Matrices", fontsize=18, fontweight="bold")
        
    plt.tight_layout()
    if(os.path.exists(image_path)):
        plt.savefig(os.path.join(image_path,"Confusion_Matrices.png"), dpi=300, bbox_inches='tight')
    plt.show()

def plot_discomfort_detection(df, truth_col, pmv_col, orig_col, mod_col):
    """
    @brief Generates a grouped bar chart for Class 0 (Discomfort) metrics.
    @param df Pandas DataFrame containing classification results.
    @param truth_col String name of ground truth column.
    @param pmv_col String name of PMV prediction column.
    @param orig_col String name of Original Two-Node prediction column.
    @param mod_col String name of ML Modified Two-Node prediction column.
    @return None
    """
    df_clean = df.dropna(subset=[truth_col, pmv_col, orig_col, mod_col])
    y_true = df_clean[truth_col]
    
    met_pmv = sklearn.metrics.precision_recall_fscore_support(y_true, df_clean[pmv_col], labels=[0, 1])
    met_orig = sklearn.metrics.precision_recall_fscore_support(y_true, df_clean[orig_col], labels=[0, 1])
    met_mod = sklearn.metrics.precision_recall_fscore_support(y_true, df_clean[mod_col], labels=[0, 1])
    
    # Extract Class 0 metrics (Index 0 of returned arrays)
    class0_pmv = [met_pmv[0][0], met_pmv[1][0], met_pmv[2][0]]
    class0_orig = [met_orig[0][0], met_orig[1][0], met_orig[2][0]]
    class0_mod = [met_mod[0][0], met_mod[1][0], met_mod[2][0]]
    
    labels = ['Precision', 'Recall', 'F1-Score']
    x = np.arange(len(labels))
    width = 0.25
    
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(x - width, class0_pmv, width, label='Baseline A (PMV RP-884)', color='firebrick')
    ax.bar(x, class0_orig, width, label='Original Two-Node (DISC)', color='steelblue')
    ax.bar(x + width, class0_mod, width, label='Modified Two-Node (DISC)', color='forestgreen')
    
    ax.set_ylabel('Score', fontsize=14)
    ax.set_title('Discomfort Detection Performance (Class 0)', fontweight='bold', fontsize=16)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=14)
    ax.set_ylim(0, 1.15)
    ax.legend(loc='upper right', fontsize=13)
    ax.grid(axis='y', linestyle=':', alpha=0.6)
    ax.tick_params(axis='y', labelsize=14)

    plt.tight_layout()
    if(os.path.exists(image_path)):
        plt.savefig(os.path.join(image_path,"Discomfort_Metrics_Comparison.png"), dpi=300, bbox_inches='tight')
    plt.show()

def plot_comfort_detection(df, truth_col, pmv_col, orig_col, mod_col):
    """
    @brief Generates a grouped bar chart for Class 1 (Comfort) metrics.
    @param df Pandas DataFrame containing classification results.
    @param truth_col String name of ground truth column.
    @param pmv_col String name of PMV prediction column.
    @param orig_col String name of Original Two-Node prediction column.
    @param mod_col String name of ML Modified Two-Node prediction column.
    @return None
    """
    df_clean = df.dropna(subset=[truth_col, pmv_col, orig_col, mod_col])
    y_true = df_clean[truth_col]
    
    met_pmv = sklearn.metrics.precision_recall_fscore_support(y_true, df_clean[pmv_col], labels=[0, 1])
    met_orig = sklearn.metrics.precision_recall_fscore_support(y_true, df_clean[orig_col], labels=[0, 1])
    met_mod = sklearn.metrics.precision_recall_fscore_support(y_true, df_clean[mod_col], labels=[0, 1])
    
    # Extract Class 1 metrics (Index 1 of returned arrays)
    class1_pmv = [met_pmv[0][1], met_pmv[1][1], met_pmv[2][1]]
    class1_orig = [met_orig[0][1], met_orig[1][1], met_orig[2][1]]
    class1_mod = [met_mod[0][1], met_mod[1][1], met_mod[2][1]]
    
    labels = ['Precision', 'Recall', 'F1-Score']
    x = np.arange(len(labels))
    width = 0.25
    
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(x - width, class1_pmv, width, label='Baseline A (PMV RP-884)', color='firebrick')
    ax.bar(x, class1_orig, width, label='Original Two-Node (DISC)', color='steelblue')
    ax.bar(x + width, class1_mod, width, label='Modified Two-Node (DISC)', color='forestgreen')
    
    ax.set_ylabel('Score', fontsize=14)
    ax.set_title('Comfort Detection Performance (Class 1)', fontweight='bold', fontsize=16)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=14)
    ax.set_ylim(0, 1.15)
    ax.legend(loc='upper right', fontsize=13)
    ax.grid(axis='y', linestyle=':', alpha=0.6)
    ax.tick_params(axis='y', labelsize=14)

    plt.tight_layout()
    if(os.path.exists(image_path)):
        plt.savefig(os.path.join(image_path,"Comfort_Metrics_Comparison.png"), dpi=300, bbox_inches='tight')
    plt.show()    
    
def plot_classification_dashboard(df, truth_col, pmv_col, disc_col_orig, disc_col_mod):
    plot_all_matrices(df, truth_col, pmv_col, disc_col_orig, disc_col_mod)
    plot_discomfort_detection(df, truth_col, pmv_col, disc_col_orig, disc_col_mod)
    plot_comfort_detection(df, truth_col, pmv_col, disc_col_orig, disc_col_mod)

# Execute (Make sure you generated the binary columns first!)
plot_classification_dashboard(dataset, truth_col='comfort_binary', pmv_col='pmv_comf_binary', disc_col_orig='disc_binary_orig', disc_col_mod='disc_binary_mod')

# add comparison of model skin temp outputs and two node model output

#%%
def calculate_ppd(pmv_series: pd.Series) -> pd.Series:
    pmv = pmv_series.to_numpy()
    ppd = 100.0 - 95.0 * np.exp(-(0.03353 * pmv**4 + 0.2179 * pmv**2))
    return pd.Series(ppd, index=pmv_series.index)

# 1. Generate your new PPD based on Two-Node SET
dataset['ppd_set_mod'] = calculate_ppd(dataset['pmv_two_node_mod'])
dataset['ppd_set_orig'] = calculate_ppd(dataset['pmv_two_node_orig'])

def plot_ppd_comparison(df: pd.DataFrame) -> None:
    # 1. Create Ground Truth Percentage (0 to 100%)
    # Votes 1, 2, 3 are "Dissatisfied"
    df['actual_dissatisfaction_pct'] = (df['thermal_comfort'] <= 3).astype(int) * 100

    # 2. Create Bins based on Standard PMV (-3 to +3)
    # Rounding to the nearest 0.5 creates clean, standard categories
    df['pmv_bin'] = np.round(df['pmv'] * 2) / 2

    # 3. Group and calculate the mean for each metric
    binned_data = df.groupby('pmv_bin').agg(
        observed_ppd=('actual_dissatisfaction_pct', 'mean'),
        baseline_ppd=('ppd', 'mean'),
        tn_mod_ppd=('ppd_set_mod', 'mean'),
        tn_orig_ppd=('ppd_set_orig', 'mean'),
    ).reset_index()

    # 4. Plotting
    plt.figure(figsize=(10, 6))

    # Ground Truth (The target we want to hit)
    plt.plot(binned_data['pmv_bin'], binned_data['observed_ppd'],
             marker='o', linestyle='-', color='black', linewidth=2.5, 
             label='Observed Dissatisfaction (RP-884)')

    # Baseline PMV 
    plt.plot(binned_data['pmv_bin'], binned_data['baseline_ppd'],
             marker='s', linestyle='--', color='red', alpha=0.7, 
             label='Baseline A PPD (Standard Model)')

    # Baseline PMV 
    plt.plot(binned_data['pmv_bin'], binned_data['tn_orig_ppd'],
             marker='s', linestyle='--', color='green', alpha=0.7, 
             label='Baseline B PPD (Original Two-Node)')

    # Modified Two-Node 
    plt.plot(binned_data['pmv_bin'], binned_data['tn_mod_ppd'],
             marker='^', linestyle='-', color='blue', linewidth=2, 
             label='Modified Two-Node PPD')

    # Formatting
    plt.title('PPD Comparison: Baseline A & B vs. Modified Two-Node vs. Reality', fontsize=16, fontweight='bold')
    plt.xlabel('Standard PMV Bins', fontsize=14)
    plt.ylabel('Percentage Dissatisfied (%)', fontsize=14)
    plt.legend(fontsize=13)
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    
    # Standard PPD boundaries
    plt.ylim(0, 100)
    plt.xlim(-3.5, 3.5)

    plt.tight_layout()

    # dpi=300 is publication quality. 
    # bbox_inches='tight' prevents labels from being cut off in the saved file.
    if(os.path.exists(image_path)):
        plt.savefig(os.path.join(image_path,"PPD_Comparison.png"), dpi=300, bbox_inches='tight')


    plt.show()

# Execute (Ensure dataset has 'ppd_set' calculated from the previous step)
plot_ppd_comparison(dataset)

# %%
