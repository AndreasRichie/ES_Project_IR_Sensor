"""!
@file Linear_Regression.py
@brief Train, evaluate and compare Multiple Linear Regression models.
@details
Read datasets → split train/test.
Iterate parameters → compile/fit models.
Evaluate models → rank by RMSE.
Save best model.
Plot predictions vs targets.
"""

# %% import libs
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import tensorflow as tf
import os
import math
from tensorflow.python import keras
from keras import layers

# Make NumPy printouts easier to read.
np.set_printoptions(precision=3, suppress=True)
print(tf.version.VERSION)
print(np.version.version)
print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))

# %% prepare, load and split datasets
# specify paths and file names
# @var path_to_datasets
# @brief Path → Datasets directory.
path_to_datasets = os.path.join(os.getcwd(), "Datasets")

# @var train_name
# @brief Filename → Train dataset.
train_name = "Dataset_Train.csv"

# @var train_file
# @brief Full path → Train dataset.
train_file = os.path.join(path_to_datasets, train_name)

# @var test_name
# @brief Filename → Test dataset.
test_name = "Dataset_Test.csv"

# @var test_file
# @brief Full path → Test dataset.
test_file = os.path.join(path_to_datasets, test_name)

# check if there is already a file with complete train and test data sets to ensure same sets for all models
if os.path.exists(train_file) & os.path.exists(test_file):
    # @var train_dataset
    # @brief DataFrame → Training data.
    train_dataset = pd.read_csv(train_file)

    # @var test_dataset
    # @brief DataFrame → Testing data.
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
# @var train_features
# @brief DataFrame → Model inputs (train).
train_features = train_dataset.copy()

# @var test_features
# @brief DataFrame → Model inputs (test).
test_features = test_dataset.copy()

# extract targets (output of model for train/test)
# @var train_targets
# @brief Series → Target output "skin_temp" (train).
train_targets = train_features.pop("skin_temp")

# @var test_targets
# @brief Series → Target output "skin_temp" (test).
test_targets = test_features.pop("skin_temp")

# %% train models with different parameters to compare them and find best possible fit and print train statistics


def compile_and_fit(model, learning_rate, loss, train_features, train_targets, validation_split):
    """!
    @brief Compile and train model.
    @details
    Optimizer = Adam(learning_rate).
    Metric = RMSE.
    Epochs = 10.
    Verbose = 1.
    @param model Keras Sequential model.
    @param learning_rate Learning rate (float).
    @param loss Loss function (str).
    @param train_features Training inputs (DataFrame).
    @param train_targets Training labels (Series).
    @param validation_split Validation fraction (float).
    @return Training history (keras.callbacks.History).
    """
    # set optimizer, loss function and metrics to check to model
    model.compile(
        optimizer=keras.optimizers.Adam(learning_rate=learning_rate),
        loss=loss, metrics=[keras.metrics.RootMeanSquaredError()])

    # train model, use verbose=1 to show progress bar
    history = model.fit(
        train_features,
        train_targets,
        epochs=10,
        verbose=1,
        validation_split=validation_split)
    return history


# @var plot_training
# @brief Boolean → Enable/disable training plots.
plot_training = False

# normalizing layer
# @var normalizer
# @brief Keras Normalization layer → Scale input features.
normalizer = layers.Normalization(axis=-1, input_shape=(3,))
normalizer.adapt(np.array(train_features))

# dicts with key: (loss, lr, vs) and value history/model
# @var all_histories
# @brief Dict → key=(loss, lr, vs), value=History.
all_histories = {}

# @var all_models
# @brief Dict → key=(loss, lr, vs), value=Model.
all_models = {}

# parameters to compare
# losses = ['mean_absolute_error', 'mean_squared_error']
# learning_rates = [0.005, 0.01, 0.05, 0.1, 0.2]
# validation_splits = [0.1, 0.2, 0.3]
losses = ['mean_squared_error']
learning_rates = [0.01]
validation_splits = [0.1]

step = 1
step_max = len(losses) * len(learning_rates) * len(validation_splits)
# train all models by iterating through parameters
for loss in losses:
    for learning_rate in learning_rates:
        for validation_split in validation_splits:
            print("Train model " + str(step) + " of " + str(step_max) + ", Parameters: loss function " +
                  loss + ", learning rate " + str(learning_rate) + ", validation split " + str(validation_split))
            # create model
            linear_model = tf.keras.Sequential(
                [normalizer, layers.Dense(units=1)])
            # train model
            all_histories[(loss, learning_rate, validation_split)] = compile_and_fit(
                linear_model, learning_rate, loss, train_features, train_targets, validation_split)
            # add model to dict
            all_models[(loss, learning_rate, validation_split)] = linear_model
            step += 1

if plot_training:
    # subplot for each LR that shows loss (full) and valloss (dotted) for each VS together
    lr_horizontal = 2
    lr_vertical = math.ceil(len(learning_rates)/lr_horizontal)
    # subplot for each VS that shows loss (full) and valloss (dotted) for each LR together
    vs_horizontal = 2
    vs_vertical = math.ceil(len(validation_splits)/vs_horizontal)
    lr_axs_idx = 0
    vs_axs_idx = 0
    for loss in losses:
        # subplots and figure for LR
        lr_fig, lr_axs = plt.subplots(
            lr_vertical, lr_horizontal, sharex='all', sharey='all')
        # subplots and figure for VS
        vs_fig, vs_axs = plt.subplots(
            vs_vertical, vs_horizontal, sharex='all', sharey='all')
        for learning_rate in learning_rates:
            lr_idx = learning_rates.index(learning_rate)
            for validation_split in validation_splits:
                vs_idx = validation_splits.index(validation_split)
                history = all_histories[(
                    loss, learning_rate, validation_split)]
                # subplot index depending on list index for LR
                lr_axs_idx = (math.floor(lr_idx/lr_horizontal),
                              lr_idx % lr_horizontal)
                # plot to subplot
                lr_axs[lr_axs_idx].plot(history.history['loss'],
                                        label="loss VS: " + str(validation_split))
                lr_axs[lr_axs_idx].plot(history.history['val_loss'],
                                        label="val_loss VS: " + str(validation_split), linestyle='dashed')
                # subplot index depending on list index for VS
                vs_axs_idx = (math.floor(vs_idx/vs_horizontal),
                              vs_idx % vs_horizontal)
                # plot to subplot
                vs_axs[vs_axs_idx].plot(history.history['loss'],
                                        label="loss LR: " + str(learning_rate))
                vs_axs[vs_axs_idx].plot(history.history['val_loss'],
                                        label="val_loss LR: " + str(learning_rate), linestyle='dashed')
                # set subplot settings VS
                vs_axs[vs_axs_idx].set_title("VS: " + str(validation_split))
                vs_axs[vs_axs_idx].grid(True)
            # set subplot settings LR
            lr_axs[lr_axs_idx].set_title("LR: " + str(learning_rate))
            lr_axs[lr_axs_idx].grid(True)
        # set figure settings LR
        lr_fig.supxlabel('Epoch')
        lr_fig.supylabel('Error [Skin Temperature]')
        lr_fig.suptitle("Loss " + loss)
        # set legend to lower right corner
        lr_handles, lr_labels = lr_axs[lr_axs_idx].get_legend_handles_labels()
        lr_fig.legend(lr_handles, lr_labels, loc="lower right")
        # set figure settings VS
        vs_fig.supxlabel('Epoch')
        vs_fig.supylabel('Error [Skin Temperature]')
        vs_fig.suptitle("Loss " + loss)
        # set legend to lower right corner
        vs_handles, vs_labels = vs_axs[vs_axs_idx].get_legend_handles_labels()
        vs_fig.legend(vs_handles, vs_labels, loc="lower right")


# %% evaluate each model and sort them by RMSE and print sorted
# @var all_evaluations
# @brief Dict → key=(loss, lr, vs), value=evaluation data.
all_evaluations = {}
step = 1
step_max = len(losses) * len(learning_rates) * len(validation_splits)
# evaluate all models
for loss in losses:
    for learning_rate in learning_rates:
        for validation_split in validation_splits:
            print("Evaluate model " + str(step) + " of " + str(step_max) + ", Parameters: loss function " +
                  loss + ", learning rate " + str(learning_rate) + ", validation split " + str(validation_split))
            model_key = (loss, learning_rate, validation_split)
            all_evaluations[model_key] = all_models[model_key].evaluate(
                test_features, test_targets, verbose=1)
            step += 1

# sort all evaluated models by RMSE, lowest first
# @var evals_sorted_rmse
# @brief List → Models sorted by RMSE ascending.
evals_sorted_rmse = sorted(all_evaluations.items(), key=lambda e: e[1][1])

# get all bar names by combining model parameters
names_rmse = [", ".join([item[0][0], "LR: " + str(item[0][1]), "VS: " + str(item[0][2])])
              for item in evals_sorted_rmse]
# get corresponding values
values_rmse = [item[1][1] for item in evals_sorted_rmse]
# print bar plot and invert y-axis so best is on top
plt.barh(names_rmse, values_rmse)
plt.gca().invert_yaxis()
plt.title("Ranking of RMSE for Parameters")
plt.xlabel("Value of RMSE")
plt.show()

# %% save best model
# @var best_model
# @brief Keras model → Lowest RMSE model.
best_model = all_models[evals_sorted_rmse[0][0]]

# save best model as Tensorflow/Keras model
path_to_models = os.path.join(os.getcwd(), "Saved_Models")
model_save_name = "MLR_Model.keras"
best_model.save(os.path.join(path_to_models, model_save_name))

# %% make predictions with best model
# @var test_predictions
# @brief Array → Predictions for test set.
test_predictions = best_model.predict(test_features).flatten()

# show comparison how well predicted with actual values correlate
a = plt.axes(aspect='equal')
plt.scatter(test_targets, test_predictions)
plt.xlabel('True Values [Skin Temperature]')
plt.ylabel('Predictions [Skin Temperature]')
lims = [20, 40]
plt.xlim(lims)
plt.ylim(lims)
_ = plt.plot(lims, lims)
plt.show()

# show histogram of difference between predictions and actual values
error = test_predictions - test_targets
plt.hist(error, bins=25)
plt.xlabel('Prediction Error [Skin Temperature]')
_ = plt.ylabel('Count')
plt.show()

# show boxplot of difference between predictions and actual values
plt.boxplot(error, labels=["MLR"])
plt.ylabel('Prediction Error [Skin Temperature]')
plt.grid(True)
plt.show()
