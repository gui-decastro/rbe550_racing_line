#!/usr/bin/env python3

# import rospy
import cv2 as cv
import numpy as np
import yaml
import os

def preprocess_image(image_path):
    # Load the image
    image = cv.imread(image_path, cv.IMREAD_GRAYSCALE)
    # cv.imshow("preprocessed_image", image)
    # cv.waitKey(0)

    # Upscale the image (if the original image size is too small)
    scale_factor = 4
    image = cv.resize(image, None, fx=scale_factor, fy=scale_factor, interpolation=cv.INTER_LINEAR)


    threshold_value = 50
    _, thresholded = cv.threshold(image, threshold_value, 255, cv.THRESH_BINARY)
    # cv.imshow("after threshold", thresholded)
    # cv.waitKey(0)

    # Invert the binary image (so free space is white and everything else is black)
    final_image = cv.bitwise_not(thresholded)

    processed_image_path = "/catkin_ws/src/map_generator/occupancy_grids/track2_occ_grid.png"
    cv.imwrite(processed_image_path, final_image)

    return image, processed_image_path

def generate_yaml(image_path, resolution, origin):
    # Preprocess the image
    processed_image, processed_image_path = preprocess_image(image_path)

    # Get image dimensions
    height, width = processed_image.shape

    # Create YAML data
    yaml_data = {
        'image': processed_image_path,
        'resolution': resolution,
        'origin': origin,
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }

    # Save the YAML data to a file
    with open('/catkin_ws/src/map_generator/racetracks/track2.yaml', 'w') as yaml_file:
        yaml.dump(yaml_data, yaml_file, default_flow_style=False)

if __name__ == "__main__":
    # for debugging:
    print(f"Current Working Directory: {os.getcwd()}")

    # image_path = rospy.get_param("~image_path", default="")
    image_path = '/catkin_ws/src/map_generator/racetracks/track2.png'
    print(f"Inside generate_map_yaml.py\nimage_path: {image_path}")
    resolution = 0.050000
    origin = [-10.000000, -10.000000, 0.000000]

    generate_yaml(image_path, resolution, origin)