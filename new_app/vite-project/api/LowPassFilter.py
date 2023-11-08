import numpy as np
from scipy.signal import butter, lfilter

class LowPassFilter:

    def __init__(self, cutoff_frequency, sampling_frequency, dims, order=5, buffer_size=300):
        self.sampling_frequency = sampling_frequency
        self.cutoff_frequency = cutoff_frequency
        self.order = order
        self.dims = dims
        self.buffer_size = buffer_size
        self.buffered_data = np.empty((0, dims))
        self.b, self.a = butter(self.order, self.cutoff_frequency / (self.sampling_frequency / 2), btype='low')

    def filter(self, data):
        self.buffered_data = np.vstack((self.buffered_data, data[np.newaxis]))

        filtered_data = np.apply_along_axis(lambda x: lfilter(self.b, self.a, x), axis=0, arr=self.buffered_data)
    
        if self.buffered_data.shape[0] >= self.buffer_size:
            self.buffered_data = self.buffered_data[-self.buffer_size//2:]  # Keep the last half of the buffered data for the next filtering iteration

        return filtered_data[-1]