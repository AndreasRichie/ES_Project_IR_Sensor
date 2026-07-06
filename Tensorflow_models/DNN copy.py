# %% import libs
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import tensorflow as tf
import os

# Make NumPy printouts easier to read.
np.set_printoptions(precision=3, suppress=True)
print(tf.__version__)
gpus = tf.config.list_physical_devices('GPU')
print("Num GPUs Available: ", len(gpus))
if gpus:
    tf.config.experimental.set_memory_growth(gpus[0], True)

# keras must be imported after gpu settings
import keras_tuner as kt 
from tensorflow import keras
from keras import layers

keras.mixed_precision.set_global_policy("mixed_float16")

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
train_labels = train_features.pop("skin_temp")
test_labels = test_features.pop("skin_temp")


def make_normalizer(train_features: np.ndarray) -> layers.Normalization:
    """!
    @brief  Create and adapt a fresh Normalization layer per model.
            Must be called once per model — a layer can only belong to one graph.

    @param  train_features  np.ndarray of shape (N, 3): [T_air, RH, MRT].
    @return Adapted Normalization layer, ready to embed in a Sequential model.
    """
    norm = layers.Normalization(axis=-1, input_shape=(3,))
    norm.adapt(train_features)
    return norm


def create_model(train_features: np.ndarray,
                 layer_count: int,
                 neurons: int,
                 l2_factor: float,
                 dropout: float,
                 ) -> tf.keras.Sequential:
    """!
    @brief  Build a Sequential DNN for skin temperature regression.

    @param  train_features  Raw training inputs — used to build a fresh normalizer.
    @param  layer_count     Number of hidden Dense layers.
    @param  neurons         Units per hidden Dense layer.
    @param  l2_factor       L2 weight decay strength; None → disabled.
    @param  dropout         Dropout rate in (0, 1); None → disabled.
                            Applied *after* each hidden Dense layer.

    @return Uncompiled tf.keras.Sequential model.

    @note   'he_normal' initializer is used instead of 'normal':
            it scales variance by 2/fan_in, which is correct for ReLU.
    @note   A new Normalization layer is created per call to avoid
            sharing a single layer across multiple model graphs.
    """

    # --- Input normalization (fresh instance per model) ---
    normalizer = make_normalizer(train_features)
    model = tf.keras.Sequential([
        normalizer
    ])

    # add hidden layers
    for _ in range(layer_count):
        regularizer = keras.regularizers.l2(
            l2_factor) if l2_factor > 0 else None

        model.add(
            layers.Dense(
                neurons,
                kernel_initializer="he_normal",
                kernel_regularizer=regularizer,
            )
        )
        model.add(layers.BatchNormalization())
        model.add(layers.Activation("relu"))
    if dropout > 0:
        model.add(layers.Dropout(dropout))

    # --- Output layer: single scalar, linear activation ---
    model.add(layers.Dense(1, kernel_initializer='normal',
              activation='linear', dtype="float32"))

    return model


def build_model(hp):
    """!
    @brief  Model builder for Keras Tuner.
            hp object replaces manual parameter lists.

    @param  hp  HyperParameters object injected by the tuner.
    @return Compiled tf.keras.Sequential model.
    """
    # neurons = hp.Choice("neurons", [512, 1024, 2048, 4096])
    neurons = hp.Choice("neurons", [128, 256, 512])
    layers_n = hp.Choice("layers", [2, 3, 4])
    l2 = hp.Choice("l2", [0.0, 1e-4, 1e-3])
    drop = hp.Choice("dropout", [0.0, 0.1, 0.2])
    # neurons = hp.Choice("neurons", [2048])
    # layers_n = hp.Choice("layers", [4])
    # l2 = hp.Choice("l2", [0.0])
    # drop = hp.Choice("dropout", [0.0])
    # lr = hp.Float("lr", min_value=1e-3, max_value=1e-1, sampling="log")
    lr = hp.Float("lr", min_value=1e-4, max_value=1e-2, sampling="log")

    model = create_model(train_features, layers_n, neurons, l2, drop)
    model.compile(
        optimizer=keras.optimizers.Adam(lr),
        loss="mean_squared_error",
        metrics=[keras.metrics.RootMeanSquaredError()]
    )
    return model


class TunableBatchTuner(kt.Hyperband):
    """!
    @brief  Hyperband tuner extended to include batch_size
            as a searchable hyperparameter.
            batch_size is injected into run_trial since it
            belongs to fit(), not build().
    """

    def run_trial(self, trial, *args, **kwargs):
        """!
        @brief  Override run_trial to inject batch_size from hp.

        @param  trial   Keras Tuner Trial object.
        @param  args    Positional args forwarded to fit().
        @param  kwargs  Keyword args forwarded to fit();
                        batch_size is overwritten here.
        """
        hp = trial.hyperparameters
        kwargs["batch_size"] = hp.Choice("batch_size", [256, 512, 1024])
        return super().run_trial(trial, *args, **kwargs)


tuner = kt.Hyperband(
    build_model,
    objective=kt.Objective("val_root_mean_squared_error", direction="min"),
    max_epochs=300,
    factor=3,
    hyperband_iterations=1,
    directory=os.path.join(os.getcwd(), "DNN_Tuner_Results2"),
    project_name="skin_temp"
)

#%%
# for very thorough visualization callbacks=[keras.callbacks.TensorBoard("./tb_logs")]
tuner.search(
    train_features, train_labels,
    validation_split=0.1,
    batch_size=512,
    callbacks=[keras.callbacks.EarlyStopping(
        monitor="val_loss",
        patience=10,
        min_delta=1e-2,
        restore_best_weights=True
    )],
    verbose=1   # shows epoch progress per trial
)

# %%
tuner.results_summary()
best_model = tuner.get_best_models(1)[0]
# %%
# save best model as Tensorflow/Keras model
path_to_models = os.path.join(os.getcwd(), "Saved_Models")
model_save_name = "DNNR_Model.keras"
best_model.save(os.path.join(path_to_models, model_save_name))

# %% make predictions with best model
test_predictions = best_model.predict(test_features).flatten()

# show comparison how well predicted with actual values correlate
a = plt.axes(aspect='equal')
plt.scatter(test_labels, test_predictions)
plt.xlabel('True Values [Skin Temperature]')
plt.ylabel('Predictions [Skin Temperature]')
lims = [20, 40]
plt.xlim(lims)
plt.ylim(lims)
_ = plt.plot(lims, lims)
plt.show()

# show histogram of difference between predictions and actual values
error = test_predictions - test_labels
plt.hist(error, bins=25)
plt.xlabel('Prediction Error [Skin Temperature]')
_ = plt.ylabel('Count')
plt.show()

# show boxplot of difference between predictions and actual values
plt.boxplot(error, labels=["DNNR"])
plt.ylabel('Prediction Error [Skin Temperature]')
plt.grid(True)
plt.show()

# %%
