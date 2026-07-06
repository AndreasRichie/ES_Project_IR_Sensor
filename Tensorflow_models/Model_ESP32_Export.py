import tensorflow as tf


def convert_to_tflite(model, train_features, output_path="skin_temp_model.tflite"):
    """
    @brief  Convert Keras model to int8 quantized TFLite.
            Uses representative dataset for full integer quantization.

    @param  model           Trained tf.keras.Sequential model.
    @param  train_features  NumPy array used to calibrate quantization ranges.
    @param  output_path     Output path for .tflite file.
    @return Quantized TFLite model bytes.
    """
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]

    def representative_data():
        for sample in tf.data.Dataset.from_tensor_slices(
                train_features.astype("float32")).batch(1).take(500):
            yield [sample]

    converter.representative_dataset = representative_data

    tflite_model = converter.convert()

    with open(output_path, "wb") as f:
        f.write(tflite_model)

    print(f"Model size: {len(tflite_model) / 1024:.2f} KB")
    return tflite_model
