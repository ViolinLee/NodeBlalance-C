import tensorflow as tf
import numpy as np
from tensorflow import keras
import tensorflow_io as tfio
from tensorflow.python.ops import gen_audio_ops as audio_ops


class AudioDetector:
    def __init__(self, h5_model_path):
        self._model = keras.models.load_model(h5_model_path)

    def _get_spectrogram(self, audio):
        # normalise the audio
        audio = audio - np.mean(audio)
        audio = audio / np.max(np.abs(audio))
        # create the spectrogram
        spectrogram = audio_ops.audio_spectrogram(audio, window_size=320, stride=160, magnitude_squared=True).numpy()
        # reduce the number of frequency bins in our spectrogram to a more sensible level
        spectrogram = tf.nn.pool(
            input=tf.expand_dims(spectrogram, -1),
            window_shape=[1, 6],
            strides=[1, 6],
            pooling_type='AVG',
            padding='SAME')
        spectrogram = tf.squeeze(spectrogram, axis=0)
        spectrogram = np.log10(spectrogram + 1e-6)
        return spectrogram

    def _get_spectrogram_from_wav(self, file_path):
        # load the audio file
        audio_tensor = tfio.audio.AudioIOTensor(file_path)

        # convert the audio to an array of floats and scale it to betweem -1 and 1
        audio = tf.cast(audio_tensor[:16000], tf.float32)

        # get the spectrogram
        return self._get_spectrogram(audio)

    def _get_spectrogram_from_nparray(self, np_arr):
        audio = tf.cast(np_arr[:16000], tf.float32)
        audio = np.expand_dims(audio, axis=1)

        # get the spectrogram
        return self._get_spectrogram(audio)

    def predict_nparray(self, np_arr):
        spectrogram = self._get_spectrogram_from_wav(np_arr)
        x = np.expand_dims(spectrogram, axis=0)
        print(np.shape(x))
        y = self._model.predict(x)[0]
        print(y)
        return y


if __name__ == '__main__':
    model_path = ''
    audio_predictor = AudioDetector(model_path)

    sample = ''
    print(audio_predictor.predict_nparray(sample))

