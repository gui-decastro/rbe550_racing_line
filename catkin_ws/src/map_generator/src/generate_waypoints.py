# Imports
import cv2
import numpy as np
import os
import pickle


def findRacetrackContours(racetrack_filename):
    """
    Find the inner and outer contours of the racetrack.
    """
    # Read the binary occupancy grid image (cv.findContours only works with a binary image)
    binary_image = cv2.imread(racetrack_filename, cv2.IMREAD_GRAYSCALE)
    # draw two lines on right and left side of the finish line to complete the racetrack and create a continous inner and outer contour
    # binary_image[144, 75:79] = [255] # found EXPERIMENTALLY (regular size)
    # binary_image[153, 75:79] = [255]
    binary_image[577, 299:317] = [255] # found EXPERIMENTALLY. (4x upscaled)
    binary_image[614, 299:317] = [255]

    # cv2.imshow('Binary Image with continous edges', binary_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Find contours of the binary image
    contours, _ = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    # Read racetrack image in color so the contours can be drawn in color
    racetrack_image = cv2.imread(racetrack_filename)

    # Filter contours based on area to get inner and outer edges
    inner_contour = []  # Need to be a list otherwise, only the points of the contours get plotted, not the contour line
    outer_contour = []

    debugging = False # set to True to help find the correct thresholds to generate an inner and outer contour

    # Set parameters 
    threshold_area = 100000
    min_area = 1000

    # Iterate through contours
    for contour in contours:
        # Create contour area
        area = cv2.contourArea(contour)
        
        # debug to find the perimeters experimentally
        if debugging:
            racetrack_image_dummy = racetrack_image.copy()
            print(f'contour area: {area}')
            cv2.drawContours(racetrack_image_dummy, contour, -1, (0, 0, 255), 2)  # Draw outer contours in red
            cv2.imshow('Racetrack Contours', racetrack_image_dummy)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        
        # Classify area by size
        if area < min_area:
            continue
        elif area < threshold_area:  # You need to define the threshold area based on your image and requirements
            print(f'area: {area}. Inner contour appended')
            inner_contour.append(contour)
        else:
            print(f'area: {area}. Outer contour appended')
            outer_contour.append(contour)

    # Extract the first list since there is only one inner and outer contour
    inner_contour = inner_contour[0]
    outer_contour = outer_contour[0]

    # Define the desired starting coordinates for outer and inner contours
    # start_outer = [74, 153] # Found EXPERIMENTALLY. Output of cv.findContours is deterministic
    # start_inner = [74, 144]
    start_outer = [298, 614] # Found EXPERIMENTALLY. This is for the 4x upscaled image
    start_inner = [298, 577]

    # Rearrange the contour so they both start at the start line and finish at the finish line
    outer_contour = rearrange_contour(outer_contour, start_outer, "outer")
    inner_contour = rearrange_contour(inner_contour, start_inner, "inner")

    # Add the contours to the racetrack image
    cv2.drawContours(racetrack_image, inner_contour, -1, (0, 255, 0), 2)  # Draw inner contours in green
    cv2.drawContours(racetrack_image, outer_contour, -1, (0, 0, 255), 2)  # Draw outer contours in red

    # Return track image and contours
    return racetrack_image, inner_contour, outer_contour


def rearrange_contour(contour, startline_coord, contour_side):
    """
    Helper function to rearrange the contour lines such that start and finish line are the first and last index.
    """
    # Convert to a numpy array to facilitate array manipulation
    contour_array = np.array(contour)

    # Find the indices of the desired starting coordinates in the arrays
    start_outer_index = np.where((contour_array[:, 0, 0] == startline_coord[0]) & (contour_array[:, 0, 1] == startline_coord[1]))[0][0]

    # Rearrange the arrays based on the starting indices
    contour = np.concatenate((contour_array[start_outer_index:], contour_array[:start_outer_index]))

    # the outer contour begins at the start line but it goes to the right instead of left, so reverse it so they both go same direction
    if contour_side == 'outer':
        contour = contour[::-1]

    # Return contours
    return contour

# 
def select_waypoints(racetrack_image, inner_contour, outer_contour, track_name):
    """
    Interactively select points on racetrack to define waypoints.
    """
    # Set variable
    global clicked_point
    clicked_point = None

    # Create a window and set the mouse callback function
    cv2.namedWindow('Interactive Racetrack Image')
    cv2.setMouseCallback('Interactive Racetrack Image', mouse_callback)
    height, _, _ = racetrack_image.shape

    # Create instructions that will be displayed on the interactive window
    text1 = "Click on the outer contour to define waypoints"
    text2 = "[s] save waypoints"
    text3 = "[Esc] abort"
    position1 = (15, 50)  # (x, y) coordinates for the text position
    position2 = (15, height - 75)
    position3 = (15, height - 25)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    color = (255, 255, 255)  # White color for the text
    thickness = 2  # Thickness of the text
    
    # Copy racetrack image for interactive window
    image_with_drawings = racetrack_image.copy()
    # Add instructions
    cv2.putText(image_with_drawings, text1, position1, font, font_scale, color, thickness)
    cv2.putText(image_with_drawings, text2, position2, font, font_scale, color, thickness)
    cv2.putText(image_with_drawings, text3, position3, font, font_scale, color, thickness)

    # Instantiate empty list
    waypoints = []
    # While waypoints are being defined
    while True:

        # Draw the clicked point if available
        if clicked_point is not None:
            # Find the closest point on the outer contour to the location of the clicked point, and draw a circle
            closest_outer_point = min(outer_contour, key=lambda pt: np.linalg.norm(np.array(pt) - np.array(clicked_point)))
            closest_outer_point = tuple(closest_outer_point[0])
            cv2.circle(image_with_drawings, closest_outer_point, 5, (0, 0, 255), -1)  # Blue circle at closest outer point

            # Find the nearest point on the inner contour
            closest_inner_point = min(inner_contour, key=lambda pt: np.linalg.norm(np.array(pt) - np.array(closest_outer_point)))
            closest_inner_point = tuple(closest_inner_point[0])
            cv2.circle(image_with_drawings, closest_inner_point, 5, (0, 255, 0), -1)  # Red circle at closest inner point

            # Draw a line connecting the points on the outer and inner edge to create a waypoint (line)
            cv2.line(image_with_drawings, tuple(closest_outer_point), tuple(closest_inner_point), (255, 0, 0), 2)  # Yellow line between points

            # Append to waypoint list
            waypoints.append((closest_outer_point, closest_inner_point))

            # reset flag and await for next click event
            clicked_point = None 

        # Show the updated image
        cv2.imshow('Interactive Racetrack Image', image_with_drawings)

        # Wait for a key press and check for exit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            print("Saving waypoints")
            # Call function to save waypoints
            save_waypoints(waypoints, track_name)
            break
        elif key == 27:  # Press Esc key to exit
            print("Abort")
            break

    # Clean up
    cv2.destroyAllWindows()


def mouse_callback(event, x, y, flags, param):
    """
    Callback function for mouse click on OpenCV window.
    """
    global clicked_point
    # If left-mouse-button click is detected
    if event == cv2.EVENT_LBUTTONDOWN:
        # Extract point
        clicked_point = (x, y)
        print(f'clicked point: {clicked_point}')


def save_waypoints(waypoints, track_name):
    """
    Save selected waypoints into a file.
    """
    # Define the file path
    base_filename = 'waypoints_data'
    extension = '.pickle'

    # Generate a unique filename
    index = 1
    while True:
        filename = f"{track_name}_{base_filename}({index}){extension}"
        if not os.path.exists(filename):
            break
        index += 1

    # Save waypoints data to the pickle file
    with open(filename, 'wb') as file:
        pickle.dump(waypoints, file)

    print(f"Waypoints data saved to {filename}")

if __name__ == "__main__":
    """
    Main function.
    """
    # Set track name and path
    track_name = 'track2'
    racetrack_filename = f'../occupancy_grids/{track_name}_occ_grid.png'
    
    # Find countours
    racetrack_image, inner_contour, outer_contour = findRacetrackContours(racetrack_filename)
    # Interactively select and save the waypoints
    select_waypoints(racetrack_image, inner_contour, outer_contour, track_name)