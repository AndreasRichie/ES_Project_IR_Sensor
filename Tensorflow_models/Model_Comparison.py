# %% import libs
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import tensorflow as tf
import os
import ydf
from sklearn.metrics import r2_score, root_mean_squared_error

# Make NumPy printouts easier to read.
np.set_printoptions(precision=3, suppress=True)

# specify paths and expected file/folder names
path_to_datasets = os.path.join(os.getcwd(), "Datasets")
test_name = "Dataset_Test.csv"
test_file = os.path.join(path_to_datasets, test_name)
path_to_models = os.path.join(os.getcwd(), "Saved_Models")
model_names = ["MLR", "GBTR", "RFR", "ANNR"]
# model_names = ["Lin_Regr", "GBT", "RF", "DNNR"]
model_save_suffix = "_Model.keras"
#%%
image_path = os.path.abspath("f:\\Dokumente\\FH\\Masterarbeit\\Master_Thesis_Reichenauer\\img")
#%%
# dict with key: model name and value: (model data, indicator if YDF model)
all_models = {}

# check for each expected model if there is one saved and if so load it
for model_name in model_names:
    model_save = os.path.join(path_to_models, model_name + model_save_suffix)
    if not os.path.exists(model_save):
        continue
    if os.path.isfile(model_save):
        all_models[model_name] = (
            tf.keras.models.load_model(model_save), False)
    else:
        all_models[model_name] = (ydf.load_model(model_save), True)

# check if there is already a file with complete test data set
if os.path.exists(test_file):
    test_dataset = pd.read_csv(test_file)
# if not read all files and create sets
else:
    # get all files
    all_files = [f for f in os.listdir(path_to_datasets) if os.path.isfile(
        os.path.join(path_to_datasets, f))]

    all_datasets = []
    # column names
    columns = ["skin_temp", "air_temp", "air_hum", "mrt"]

    # check all detected files
    for file in all_files:
        # if there is already a train or test file skip it
        if ("Test" in file) | ("Train" in file):
            continue
        # if file is not a csv file skip it
        if not (".csv" in file):
            continue
        # read file and append to dataset collection
        dataset = pd.read_csv(os.path.join(
            path_to_datasets, file), names=columns)
        all_datasets.append(dataset)

    # combine all datasets
    complete_dataset = pd.concat(all_datasets, ignore_index=True, sort=False)

    # create train and test dataset
    train_dataset = complete_dataset.sample(frac=0.8, random_state=0)
    test_dataset = complete_dataset.drop(train_dataset.index)

    # save test dataset for next use/other models
    test_dataset.to_csv(test_file, index=False)

# extract features (used as input to model while train/test)
test_features = test_dataset.copy()

# extract labels (output of model for train/test)
test_labels = test_features.pop("skin_temp")

# evaluate each model and sort them once after RMSE and once after R2 and print sorted

# dict with key: model name and value: predicted values
all_predictions = {}
# dict with key: model name and value: (RMSE, R2 score)
all_evaluations = {}
legend_labels = []
for name, model in all_models.items():
    # check if it is an YDF model, if so it must be handled differently
    if model[1]:
        predictions = model[0].predict(test_dataset)
    else:
        predictions = model[0].predict(test_features).flatten()
    all_predictions[name] = predictions
    rmse = root_mean_squared_error(test_labels, predictions)
    r2 = r2_score(test_labels, predictions)
    all_evaluations[name] = (rmse, r2)
    legend_labels.append(name)

# sort all evaluated models by RMSE, lowest first
evals_sorted_rmse = sorted(all_evaluations.items(), key=lambda e: e[1][0])
# sort all evaluated models by R2 Score, highest first
evals_sorted_r2 = sorted(all_evaluations.items(),
                         key=lambda e: e[1][1], reverse=True)

#%%

# colour schemes for models
colour_map = {
    "MLR": "tab:blue",
    "RFR": "tab:green",
    "GBTR": "tab:orange",
    "ANNR": "tab:red"
}

# get all bar names by combining model parameters
names_rmse = [item[0] for item in evals_sorted_rmse]
# get corresponding values
values_rmse = [item[1][0] for item in evals_sorted_rmse]
# get colours
mapped_colours_rmse = [colour_map.get(name, 'gray') for name in names_rmse]
# print bar plot and invert y-axis so best is on top
plt.barh(names_rmse, values_rmse, color=mapped_colours_rmse)
plt.gca().invert_yaxis()
plt.title("RMSE of all the ML Model Predictions", fontweight='bold', fontsize=16)
plt.xlabel("RMSE of $T_{sk}$ in °C", fontsize=14)
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid(True, linestyle=':', alpha=0.6)
plt.tight_layout()
if(os.path.exists(image_path)):
    plt.savefig(os.path.join(image_path,"ML_Models_RMSE.png"), dpi=300, bbox_inches='tight')
plt.show()

# get all bar names by combining model parameters
names_r2 = [item[0] for item in evals_sorted_r2]
# get corresponding values
values_r2 = [item[1][1] for item in evals_sorted_r2]
# print bar plot and invert y-axis so best is on top
# get colours
mapped_colours_r2 = [colour_map.get(name, 'gray') for name in names_r2]
plt.barh(names_r2, values_r2, color=mapped_colours_r2)
plt.gca().invert_yaxis()
plt.title(r"$\mathbf{R^2}$ of all the ML Model Predictions", fontweight='bold', fontsize=16)
plt.xlabel("Value of $R^2$", fontsize=14)
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid(True, linestyle=':', alpha=0.6)
plt.tight_layout()
if(os.path.exists(image_path)):
    plt.savefig(os.path.join(image_path,"ML_Models_R2.png"), dpi=300, bbox_inches='tight')
plt.show()

#%%
# plot predictions for all models
# show comparison how well predicted with actual values correlate
# Setup 2x2 grid
fig, axes = plt.subplots(2, 2, figsize=(12, 12))
axes = axes.flatten() 
lims = [20, 40]

# Loop through predictions and assign to subplots
for i, (label, prediction) in enumerate(zip(legend_labels, all_predictions.values())):
    ax = axes[i]
    
    # Plot data + Reference line
    ax.scatter(test_labels, prediction, color=colour_map.get(label, 'gray'))
    ax.plot(lims, lims, 'r--', linewidth=2, label='Perfect Tracking')
    
    # Formatting per subplot
    ax.set_aspect('equal')
    ax.set_xlim(lims)
    ax.set_ylim(lims)
    ax.set_title(label, fontsize=16, fontweight='bold')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.tick_params(axis='both', labelsize=14)


# Global axis labels
fig.supxlabel('True Values for $T_{sk}$ in °C', fontsize=14)
fig.supylabel('Predictions for $T_{sk}$ in °C', fontsize=14)
plt.suptitle(r'Correlation between ML Predictions and True Values for $\mathbf{T_{sk}}$', fontsize=18, fontweight='bold')

plt.tight_layout()
if(os.path.exists(image_path)):
    plt.savefig(os.path.join(image_path,"ML_Models_Scatter.png"), dpi=200, bbox_inches='tight')
plt.show()

# show histogram of difference between predictions and actual values
all_errors = []
for prediction in all_predictions.values():
    error = prediction - test_labels
    # plt.hist(error, bins=25)
    all_errors.append(error)
# plt.xlabel('Prediction Error [Skin Temperature]')
# plt.legend(legend_labels)
# _ = plt.ylabel('Count')
# plt.show()

# show boxplot of difference between predictions and actual values
plt.boxplot(all_errors, labels=legend_labels)
plt.title("Distributon of ML Model Predictions Errors", fontweight='bold', fontsize=16)
plt.ylabel('Prediction Error for $T_{sk}$ in °C', fontsize=14)
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid(True, linestyle=':', alpha=0.6)
if(os.path.exists(image_path)):
    plt.savefig(os.path.join(image_path,"ML_Models_Boxplot.png"), dpi=300, bbox_inches='tight')
plt.show()

# %%
