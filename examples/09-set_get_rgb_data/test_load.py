"""
    Python image loading of binary file 
"""
from PIL import Image
import numpy as np

# Read the binary image data
with open('../../build/examples/09-set_get_rgb_data/test.bin', 'rb') as f:
    image_data = f.read()

# Convert the binary data to a numpy array
image_array = np.frombuffer(image_data, dtype=np.uint8)

# Reshape the numpy array to the image dimensions (assuming RGBA format)
image_array = image_array.reshape((96, 96, 4))

# Flip the image vertically
image_array = np.flipud(image_array)

# Create an image from the numpy array
image = Image.fromarray(image_array, 'RGBA')

# Display the image
image.show()

