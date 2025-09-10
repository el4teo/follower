import scipy.io

def read_mat_file(filename):
    """Read and print contents of a MATLAB .mat file."""
    mat_data = scipy.io.loadmat(filename)

    print(f"Contents of {filename}:")
    for key, value in mat_data.items():
        # Skip MATLAB metadata fields that always appear
        if key.startswith("__"):
            continue
        print(f"\nVariable: {key}")
        print(value)

if __name__ == "__main__":
    # Replace with the path to your .mat file
    read_mat_file("legacy\data\CarlosV_1.mat")
