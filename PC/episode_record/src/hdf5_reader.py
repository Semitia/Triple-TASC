#!/usr/bin/python3

import h5py

def print_hdf5_structure(name, obj):
    """Recursively print the structure and data of the HDF5 file."""
    if isinstance(obj, h5py.Dataset):
        print(f"Dataset: {name}, shape: {obj.shape}, dtype: {obj.dtype}")
        #print(f"Data: {obj[()]}\n")  # print the entire dataset
    elif isinstance(obj, h5py.Group):
        print(f"Group: {name}\n")

def print_hdf5_file(file_path):
    with h5py.File(file_path, 'r') as h5file:
        h5file.visititems(print_hdf5_structure)

if __name__ == "__main__":
    for i in range(5):
        hdf5_file_path = f"/home/oskar/hp09_ws/data/hdf5/fake/episode_{i}.hdf5"  # Replace with your actual file path
        print_hdf5_file(hdf5_file_path)