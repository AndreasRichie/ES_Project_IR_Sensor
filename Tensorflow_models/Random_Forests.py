# %% import libs
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import os
import ydf

# Make NumPy printouts easier to read.
np.set_printoptions(precision=3, suppress=True)

# %% prepare, load and split datasets
# specify paths and file names
path_to_datasets = os.path.join(os.getcwd(), "Datasets")
train_name = "Dataset_Train.csv"
train_file = os.path.join(path_to_datasets, train_name)
test_name = "Dataset_Test.csv"
test_file = os.path.join(path_to_datasets, test_name)

# check if there is already a file with complete train and test data sets to ensure same sets for all models
if os.path.exists(train_file) & os.path.exists(test_file):
    train_dataset = pd.read_csv(train_file)
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

    # save train and test dataset for next use/other models
    train_dataset.to_csv(train_file, index=False)
    test_dataset.to_csv(test_file, index=False)

# print statistical data of train dataset
# sns.pairplot(train_dataset[columns], diag_kind='kde')
# train_dataset.describe().transpose()

# extract features (used as input to model while train/test)
train_features = train_dataset.copy()
test_features = test_dataset.copy()

# extract labels (output of model for train/test)
train_targets = train_features.pop("skin_temp")
test_targets = test_features.pop("skin_temp")

# %% train model, evaluate and save model
"""!
@ brief Configures random search tuner for automated hyperparameter optimization.
@ details
* Evaluates different parameter combinations to minimize regression error.
* num_trials = 50 → Trains and evaluates 50 distinct models.
@ return Configured RandomSearchTuner instance.
"""
tuner = ydf.RandomSearchTuner(num_trials=50)

"""!
@ brief Defines discrete search space for maximum tree depth.
@ param "max_depth" Hyperparameter identifier.
@ param[15, 25, 30] Candidate integer values.
"""
tuner.choice("max_depth", [15, 25, 30])

"""!
@ brief Defines discrete search space for minimum leaf examples.
@ param "min_examples" Hyperparameter identifier.
@ param[2, 5, 10] Candidate integer values.
"""
tuner.choice("min_examples", [2, 5, 10])

"""!
@ brief Defines discrete search space for number of treas.
@ param "num_trees" Hyperparameter identifier.
@ param[2, 5, 10] Candidate integer values.
"""
tuner.choice("num_trees", [200, 300, 500])

"""!
@ brief Defines discrete search space for candidate attributes ratio.
@ param "num_candidate_attributes_ratio" Hyperparameter identifier.
@ param 0.33 Lower bound.
@ param 1.0 Upper bound.
"""
tuner.choice("num_candidate_attributes_ratio", [0.33, 0.66, 1.0])

"""!
@ brief Trains RF model using the automated tuner.
@ details
* Tuner identifies best parameter combination → retrains final model automatically.
@ param train_dataset Dataset for training.
@ return Tuned GenericModel.
"""
model = ydf.RandomForestLearner(
    label="skin_temp",
    task=ydf.Task.REGRESSION,
    tuner=tuner
).train(train_dataset)

"""!
@ brief Displays model structure and tuning history.
@ details "Tuning" tab in output shows all 50 evaluated combinations + individual scores.
@ param output_format Format for Jupyter Notebook UI.
"""
model.describe(output_format="notebook")

# evaluate a model (e.g. roc, accuracy, confusion matrix, confidence intervals)
evaluation = model.evaluate(test_dataset)
print(evaluation)

# %%
# save model as Tensorflow/Keras model
path_to_models = os.path.join(os.getcwd(), "Saved_Models")
model_save_name = "RFR_Model.keras"
model = ydf.load_model(os.path.join(path_to_models, model_save_name))
model.describe(output_format="notebook")
# model.save(os.path.join(path_to_models, model_save_name))

# %%
# model.to_tensorflow_saved_model(
#     path=os.path.join(path_to_models, model_save_name), mode='keras')
# %% generate predictions
# show comparison how well predicted with actual values correlate
predictions = model.predict(test_dataset)
a = plt.axes(aspect='equal')
plt.scatter(test_targets, predictions)
plt.xlabel('True Values [Skin Temperature]')
plt.ylabel('Predictions [Skin Temperature]')
lims = [20, 40]
plt.xlim(lims)
plt.ylim(lims)
_ = plt.plot(lims, lims)
plt.show()

# show histogram of difference between predictions and actual values
error = predictions - test_targets
plt.hist(error, bins=25)
plt.xlabel('Prediction Error [Skin Temperature]')
_ = plt.ylabel('Count')
plt.show()

# show boxplot of difference between predictions and actual values
plt.boxplot(error, labels=["RFR"])
plt.ylabel('Prediction Error [Skin Temperature]')
plt.grid(True)
plt.show()

# %%
