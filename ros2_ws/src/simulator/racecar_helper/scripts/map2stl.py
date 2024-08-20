#!/usr/bin/env python3

import os
import sys
import cv2
import yaml
import numpy as np
from stl import mesh
from ament_index_python.packages import get_package_share_directory

package_name = 'racecar_helper'
base_path = get_package_share_directory(package_name)

yaml_path = sys.argv[1]

# Get the parent folder of the YAML file
map_folder = "/".join(yaml_path.split("/")[:-1])
map_name = yaml_path.split("/")[-1].split(".")[0]

print(f"Loading image from path: {yaml_path}")

# Load the YAML file
with open(yaml_path, 'r') as file:
    map_data = yaml.safe_load(file)

image_relative_path = map_data['image']
image_path = f"{map_folder}/{image_relative_path}"

# Now you can use image_absolute_path with OpenCV
image = cv2.imread(image_path)

if image is None:
    print("Failed to load image from path: {}".format(image_path))
    sys.exit(1)
else:
    print("Image loaded successfully from path: {}".format(image_path))

# Read the YAML file
with open(yaml_path, 'r') as file:
    yaml_data = yaml.safe_load(file)

resolution = yaml_data['resolution']
origin = yaml_data['origin']

# Load the image
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Get the white region
white_region = image > 200

# Get the border of the white region
white_region = white_region.astype(np.uint8)
contours, _ = cv2.findContours(white_region, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

# Define the wall height and thickness
height = 1  # Height of the wall
thickness = 0.01  # Thickness of the wall

# Convert the contours to a 3D mesh
vertices = []
faces = []

def add_face(v0, v1, v2, v3):
    faces.append([v0, v1, v2])
    faces.append([v0, v2, v3])

def process_contour(contour, base_index, origin, resolution, thickness):
    contour_vertices = []
    for point in reversed(contour):
        x, y = point[0]
        x_transformed = origin[0] + x * resolution
        y_transformed = origin[1] + (image.shape[0] - y) * resolution  # Invert y-axis
        contour_vertices.append([x_transformed, y_transformed, 0])
        contour_vertices.append([x_transformed, y_transformed, height])
    
    num_vertices = len(contour_vertices) // 2

    # Generate outer contour by translating the original contour points
    outer_contour_vertices = []
    for point in contour:
        x, y = point[0]
        x_transformed = origin[0] + (x * resolution + np.sign(x) * thickness)
        y_transformed = origin[1] + (image.shape[0] - y) * resolution + np.sign(y) * thickness  # Invert y-axis
        outer_contour_vertices.append([x_transformed, y_transformed, 0])
        outer_contour_vertices.append([x_transformed, y_transformed, height])

    # Add faces for inner contour
    for i in range(num_vertices):
        next_i = (i + 1) % num_vertices
        add_face(base_index + i * 2, base_index + next_i * 2, base_index + next_i * 2 + 1, base_index + i * 2 + 1)

    # Add faces for outer contour
    outer_base_index = base_index + len(contour_vertices)
    for i in range(num_vertices):
        next_i = (i + 1) % num_vertices
        add_face(outer_base_index + i * 2, outer_base_index + next_i * 2, outer_base_index + next_i * 2 + 1, outer_base_index + i * 2 + 1)

    return contour_vertices, outer_contour_vertices

# Process each contour separately
for contour in contours:
    base_index = len(vertices)
    contour_vertices, outer_contour_vertices = process_contour(contour, base_index, origin, resolution, thickness)
    vertices.extend(contour_vertices)
    vertices.extend(outer_contour_vertices)

vertices = np.array(vertices)
faces = np.array(faces)

# Create the mesh
wall_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
for i, face in enumerate(faces):
    for j in range(3):
        wall_mesh.vectors[i][j] = vertices[face[j]]

# # Save to STL file
# Create path if it doesn't exist
os.makedirs(f"{base_path}/models/map/meshes", exist_ok=True)
output_stl_path = f'{base_path}/models/map/meshes/map.stl'
# # Create folder if it doesn't exist
# os.makedirs(os.path.dirname(output_stl_path), exist_ok=True)
wall_mesh.save(output_stl_path)

print(f"STL file saved to {output_stl_path}")