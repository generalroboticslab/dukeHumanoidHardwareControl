import numpy as np
from collections.abc import Iterable
from typing import Union
from scipy import signal

class _BaseRingBuffer:
    def __init__(self, buffer_len: int, shape: Union[Iterable,int], dtype=np.float32):
        """
        Base class for circular numpy buffers.

        Args:
            buffer_len: Maximum number of arrays the buffer can hold.
            shape: Shape of the arrays to be stored (e.g., (3, 4), (6,)).
            dtype: The data type of the arrays in the buffer (default: np.float32).
        """
        self.buffer_len = buffer_len
        if not isinstance(shape, Iterable):
            shape = (shape,)
        # self.storage acts as a circular buffer, the element at self._step is the newest element
        self.storage = np.zeros((buffer_len, *shape), dtype=dtype)
        self.step = -1  # current step
        self.max_step = self.buffer_len - 1

    def add(self, array: np.ndarray):
        """
        Appends an array to the buffer, wrapping around if full.

        Args:
            array: The array to add to the buffer. Must have the correct shape.
        """
        self.step += 1
        self.storage[self.step % self.buffer_len] = array
        
        
    def __getitem__(self, index: Union[int, slice, Iterable]) -> np.ndarray:
        """
        Retrieves an array from the buffer by index, newest to oldest.
        For example:
          index=0 returns the latest array added to the buffer.
          index=-1 returns the oldest array added to the buffer.
          index=slice(3) returns the last three arrays added to the buffer, newest to oldest.

        Args:
            index: The index of the array to retrieve.

        Returns:
            The array at the specified index.
        """
        current_step = self.step
        if isinstance(index, slice):
            start, stop, step = index.indices(self.buffer_len)  # Concise slice handling
            indices = (current_step - np.arange(start, stop, step)) % self.buffer_len
            return self.storage[indices]
        elif isinstance(index, Iterable):
            indices = (current_step - np.array(index)) % self.buffer_len
            return self.storage[indices]
        else:
            return self.storage[(current_step - index) % self.buffer_len]
        
    def get_last(self):
        """Returns the last array in the buffer"""
        return self.storage[self.step % self.buffer_len]
    
    def get_last_n(self, n: int) -> np.ndarray:
        """Returns the last n arrays in the buffer, newest to oldest"""
        assert n <= self.buffer_len and n > 0
        # print(np.arange(self.step, self.step-n, -1) % self.buffer_len)
        return self.storage[np.arange(self.step, self.step-n, -1) % self.buffer_len]
    

class RingArrayBuffer(_BaseRingBuffer):    
    def __len__(self) -> int:
        """
        Returns the current number of elements in the buffer.

        Returns:
            The number of elements in the buffer.
        """
        return min(self.step, self.buffer_len)

    def full(self) -> bool:
        """
        Checks if the buffer is full.

        Returns:
            True if the buffer is full, False otherwise.
        """
        return self.step >= self.max_step
    
    def reset(self):
        """Resets the buffer current step to -1"""
        self.step = 0
    
    def clear(self):
        """Clears the contents of the buffer by filling it with zeros."""
        self.storage.fill(0)
        self.step = 0  # Reset the step to indicate an empty buffer




class filterBuffer:
    def __init__(self,shape,fs:float=200,filter_order:int=4,cut_off_frequency:float=20,dtype=np.float32):

        
        self.filter_order = filter_order
        self.buf_raw = RingArrayBuffer(buffer_len=filter_order+1, shape=shape,dtype=dtype) # x, input
        self.buf_filt = RingArrayBuffer(buffer_len=filter_order+1, shape=shape,dtype=dtype) # y, filtered
        
        self.b, self.a = signal.butter(N=filter_order, Wn=cut_off_frequency, btype='low', analog=False,fs=fs)
        self.b: np.ndarray = self.b.reshape(-1, *([1] * len(self.buf_raw.storage.shape[1:])))
        self.a: np.ndarray = self.a.reshape(-1, *([1] * len(self.buf_raw.storage.shape[1:])))[1:filter_order+1]
        # self.b = self.b[:,np.newaxis]
        # self.a = self.a[1:filter_order+1,np.newaxis]
        
    def add(self,sample:np.ndarray):
        self.buf_raw.add(sample) 
        filtered_sample = (self.b*self.buf_raw[:]).sum(axis=0) - \
            (self.a*self.buf_filt[0:self.filter_order]).sum(axis=0)
        self.buf_filt.add(filtered_sample)
        return filtered_sample

    def get_latest_filtered(self):
        return self.buf_filt[0].copy()
    
    def get_latest_raw(self):
        return self.buf_raw[0].copy()
    
    def reset(self):
        self.buf_raw.reset()
        self.buf_filt.reset()


#--- test

def test_ring_array_buffer():
    # Create a RingArrayBuffer with a buffer length of 5 and shape (2,)
    buffer = RingArrayBuffer(buffer_len=5, shape=(2,))

    # Add arrays to the buffer
    buffer.add(np.array([1, 2]))
    buffer.add(np.array([3, 4]))
    buffer.add(np.array([5, 6]))
    buffer.add(np.array([7, 8]))
    buffer.add(np.array([9, 10]))

    # Check if buffer is full
    assert buffer.full() == True

    # Get the last added array
    assert np.array_equal(buffer.get_last(), np.array([9, 10]))

    # Get the last 3 arrays
    assert np.array_equal(buffer.get_last_n(3), np.array([[9, 10], [7, 8], [5, 6]]))

    # Add another array, should overwrite the oldest one
    buffer.add(np.array([11, 12]))
    
    # Get the last 3 arrays
    assert np.array_equal(buffer.get_last_n(3), np.array([[11, 12], [9, 10], [7, 8]]))

    # Retrieve individual arrays
    assert np.array_equal(buffer[0], np.array([11, 12]))
    assert np.array_equal(buffer[-1], np.array([3, 4]))

    # Slice retrieval
    assert np.array_equal(buffer[:3], np.array([[11, 12], [9, 10], [7, 8]]))
    
    # Reset the buffer
    buffer.reset()
    # print(buffer.get_last())
    np.array_equal(buffer.get_last(),np.array([ 9., 10.]))

    # Clear the buffer
    buffer.clear()
    assert np.array_equal(buffer.get_last(), np.array([0, 0]))

    print("All tests passed!")

if __name__ == "__main__":
    test_ring_array_buffer()