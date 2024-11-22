import yaml
import cv2
import numpy as np

# Path to the YAML file
yaml_file = "/home/vinayak/my_map.yaml"

# Load the YAML file
with open(yaml_file, 'r') as file:
    map_data = yaml.safe_load(file)

# Extract map information
image_file = map_data['image']
resolution = map_data['resolution']
origin = map_data['origin']

# Load the map image
map_image = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)

# Threshold to identify free space (assumes 254 or 255 is free space)
free_space_threshold = 254

# Find all free space pixels
free_space_pixels = np.where(map_image >= free_space_threshold)

# Convert pixel indices to coordinates
free_space_coordinates = []
for y, x in zip(free_space_pixels[0], free_space_pixels[1]):
    # Convert pixel index to world coordinates
    wx = origin[0] + x * resolution
    wy = origin[1] + y * resolution
    free_space_coordinates.append((wx, wy))

# Print or save the free space coordinates
for coord in free_space_coordinates:
    print(coord)

# Optionally, save the coordinates to a file
with open('/home/vinayak/free_space_coordinates.txt', 'w') as f:
    for coord in free_space_coordinates:
        f.write(f"{coord[0]}, {coord[1]}\n")
