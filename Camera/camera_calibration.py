# Import required modules
import cv2
import numpy as np
import os
import glob


# Define the dimensions of checkerboard
# how much corners can be found:
# horizontal and vertical
# also size of the side of a square in [mm])
CHECKERBOARD = (8, 5, 251/9)


# stop the iteration when specified
# accuracy, epsilon, is reached or
# specified number of iterations are completed.
criteria = (cv2.TERM_CRITERIA_EPS +
            cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# Vector for 3D points
threedpoints = []

# Vector for 2D points
twodpoints = []


#  3D points real world coordinates
# creating array for each corner [[x1,y1,z1],[x2,y2,z2],...,[xn,yn,zn]]^T
objectp3d = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0],
                               0:CHECKERBOARD[1]].T.reshape(-1, 2)*CHECKERBOARD[2]

# Extracting path of individual image stored
# in a given directory. Since no path is
# specified, it will take current directory
images = glob.glob('./Camera/captured_frames/*.png')

print(images)

for filename in images:

    # read image
    image = cv2.imread(filename)

    # change colors into black and white
    grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    # If desired number of corners are
    # found in the image then ret = true
    ret, corners = cv2.findChessboardCorners(
        grayColor, (CHECKERBOARD[0], CHECKERBOARD[1]), None)

    # If desired number of corners can be detected then,
    # refine the pixel coordinates and display
    # them on the images of checker board
    if ret == True:
        threedpoints.append(objectp3d)

        # Refining pixel coordinates
        # for given 2d points.
        corners2 = cv2.cornerSubPix(
            grayColor, corners, (11, 11), (-1, -1), criteria)

        twodpoints.append(corners2)

        # Draw and display the corners
        image = cv2.drawChessboardCorners(
            image, (CHECKERBOARD[0], CHECKERBOARD[1]), corners2, ret)

    cv2.imshow('png', image)
    cv2.waitKey(0)


cv2.destroyAllWindows()

h, w = image.shape[:2]

# Perform camera calibration by
# passing the value of above found out 3D points (threedpoints)
# and its corresponding pixel coordinates of the
# detected corners (twodpoints)
ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
    threedpoints, twodpoints, grayColor.shape[::-1], None, None)

# save calibration into file
with open('./Camera/camera_calibration.npy', 'wb') as f:
    np.save(f, matrix)
    np.save(f, distortion)

# Displaying required output
print(" Camera matrix:")
print(matrix)

print("\n Distortion coefficient:")
print(distortion)

print("\n Rotation Vectors:")
print(r_vecs)

print("\n Translation Vectors:")
print(t_vecs)
