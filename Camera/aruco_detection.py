import cv2
from cv2 import aruco
import numpy as np
import os
import json
import math
from threading import Thread
import argparse
from pynput import keyboard
from RPI_client import RPI_Communication_Client
import socket

# This is main function for aruco markers detection.
# It contains detection, saving into jsons files and
# sending jsons to server with sockets.


# --------------USER SETTINGS---------------

server_IP = "192.168.12.120"
server_PORT = 9999

# camera setting:
resolution = [640, 480]
FPS = 40

# table real sizes;
# assumption that table is rectangle and longer side is on x axis (width on photo)
# enter real-world values [mm] for side where 3rd, 2nd markers and 3rd, 0th markers are:
TABLE_SIZE = {'long side': 1800, 'short side': 1400}

# define which markers you use as robots, obstacles, and corners:
robots_markers_ids = [13, 14, 15]
obstacles_markers_ids = [4, 5, 6, 7, 8, 9, 10, 11, 12]
area_markers_ids = [0, 1, 2, 3]

# define real size of a side of markers (in [mm]):
marker_real_size = 335


# -----------END OF USER SETTING-----------


# construct the argument parse and parse the arguments
# (these will be argument that you can use while calling aruco_detection.py):
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--display", type=int, default=1,
                help="Whether or not frames should be displayed")
ap.add_argument("-js", "--json_sending", type=int, default=1,
                help="Whether or not you want to stream json files")
args = vars(ap.parse_args())


class Aruco_markers():
    def __init__(self, marker_Size=3, total_Markers=20):

        self.bbox = {}
        self.marker_Size = marker_Size
        self.total_Markers = total_Markers
        self.ids: int = []
        self.rejected = None
        self.coordinates = {}
        self.orientations = {}
        self.coordinates_estimated = {}
        self.orientations_estimated = {}
        self.augment_images_dict: dict = None
        self.aruco_Cam_param = None
        self.obstacles_list_ids = obstacles_markers_ids
        self.area_list_ids = area_markers_ids
        self.robots_list_ids = robots_markers_ids
        self.detected_corners = {}
        self.detected_robots = {}
        self.detected_obstacles = {}
        self.X_shift = 0
        self.Y_shift = 0

        # for simple estimation real coordinates:
        # X = (pixels_x - px_shift_x) * px_resize_x
        # Y = (pixels_y - px_shift_y) * px_resize_y
        # where
        # px_shift_x = left_bottom_corner.pixels_x
        # px_shift_y = left_bottom_corner.pixels_y
        # px_resize_x = TABLE_SIZE[long side] / (left_bottom_corner.pixels_x - right_bottom_corner.pixels_x)
        # px_resize_y = TABLE_SIZE[short side] / (left_bottom_corner.pixels_y - left_top_corner.pixels_y)

        self.coordinates_px_shift_x = 0
        self.coordinates_px_shift_y = 0
        self.coordinates_px_resize_x = 1
        self.coordinates_px_resize_y = 1

        # size of a side of aruco marker [mm]
        self.marker_real_size = marker_real_size

    # ----- adding images on aruco markers ----
    def load_Aug_Images(self, path):
        '''
        :param path: folder in which markers images are stored
        :return: dictionary with key as the id and values as the augment image
        '''
        myList = os.listdir(path)
        aug_Dics = {}
        for imgPath in myList:
            key = int(os.path.splitext(imgPath)[0])
            imgAug = cv2.imread(f'{path}/{imgPath}')
            aug_Dics[key] = imgAug
        self.augment_images_dict = aug_Dics

    def augment_Aruco(self, img, drawId=True):

        for id in self.ids:
            id = int(id)
            if id in self.augment_images_dict.keys():

                top_left = int(self.bbox[id][0, 0]), int(self.bbox[id][0, 1])
                top_right = int(self.bbox[id][1, 0]), int(self.bbox[id][1, 1])
                bottom_right = int(self.bbox[id][2, 0]), int(
                    self.bbox[id][2, 1])
                bottom_left = int(self.bbox[id][3, 0]), int(
                    self.bbox[id][3, 1])

                h, w, c = self.augment_images_dict[int(id)].shape

                pts1 = np.array(
                    [top_left, top_right, bottom_right, bottom_left])
                pts2 = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
                matrix, _ = cv2.findHomography(pts2, pts1)
                imgOut = cv2.warpPerspective(
                    self.augment_images_dict[int(id)], matrix, (img.shape[1], img.shape[0]))
                cv2.fillConvexPoly(img, pts1.astype(int), (0, 0, 0))
                imgOut = img + imgOut

                if drawId:
                    cv2.putText(imgOut, str(id), top_left,
                                cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)

                img = imgOut

        return img

    # ----- finding aruco markers ----
    def find_Aruco_Markers(self, img, draw=True):
        '''
        :param img: img in which to find the aruco markers
        :param draw: flag to draw bbox around markers detected
        :return: True if any markers found False if not
        '''

        img_Gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if (self.marker_Size <= 3):
            aruco_Dict = aruco.custom_dictionary(
                self.total_Markers, self.marker_Size, 2020)
        else:
            key = getattr(
                aruco, f'DICT_{self.marker_Size}X{self.marker_Size}_{self.total_Markers}')
            aruco_Dict = aruco.Dictionary_get(key)

        bbox, self.ids, self.rejected = aruco.detectMarkers(
            img_Gray, aruco_Dict, self.aruco_Cam_param)
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
            bbox, self.marker_real_size, self.aruco_Cam_param[0], self.aruco_Cam_param[1])

        if self.ids is not None:

            self.ids = self.ids.flatten()
            aruco.drawDetectedMarkers(img, bbox)

            for i, id in enumerate(self.ids):

                # iterate all ids and save all dictionares
                self.bbox[id] = bbox[i][0]

                rotation_matrix, _ = cv2.Rodrigues(-1*rvec[i][0])
                # np.dot(rotation_matrix, -1*tvec[i][0])
                self.coordinates_estimated[id] = tvec[i][0]

                self.orientations[id] = rotationMatrixToEulerAngles(
                    rotation_matrix)
                self.coordinates[id] = [np.sum(self.bbox[id][:, 0])/4,
                                        np.sum(self.bbox[id][:, 1])/4]

                if id in self.area_list_ids:
                    # save for detected corners coordinates x,y
                    self.detected_corners[id] = self.coordinates[id][0:2]

                if len(self.detected_corners) >= 3:
                    self.X_shift = min(
                        self.coordinates_estimated[id][0] for id in self.detected_corners.keys())
                    self.Y_shift = min(
                        self.coordinates_estimated[id][1] for id in self.detected_corners.keys())
                    self.coordinates_px_shift_x = min(
                        self.detected_corners[id][0] for id in self.detected_corners.keys())
                    self.coordinates_px_shift_y = min(
                        self.detected_corners[id][1] for id in self.detected_corners.keys())
                    self.coordinates_px_resize_x = TABLE_SIZE["long side"]/(max(
                        self.detected_corners[id][0] for id in self.detected_corners.keys()) - self.coordinates_px_shift_x)
                    self.coordinates_px_resize_y = TABLE_SIZE["short side"]/(max(
                        self.detected_corners[id][1] for id in self.detected_corners.keys()) - self.coordinates_px_shift_y)

            for id in self.ids:
                # update coordinates after calculating corrections
                self.coordinates[id] = [(self.coordinates[id][0] - self.coordinates_px_shift_x) * self.coordinates_px_resize_x,
                                        (self.coordinates[id][1] - self.coordinates_px_shift_y) * self.coordinates_px_resize_y]
                self.coordinates_estimated[id] = [
                    self.coordinates_estimated[id][0] - self.X_shift, self.coordinates_estimated[id][1] - self.Y_shift]

                if id in self.area_list_ids:
                    # save for detected corners coordinates x,y
                    self.detected_corners[id] = self.coordinates[id][0:2]

                elif id in self.robots_list_ids:
                    # save for detected robots coordinates x,y and rotation in z
                    self.detected_robots[id] = np.append(
                        self.coordinates[id][0:2], self.orientations[id][2])

                elif id in self.obstacles_list_ids:
                    # save for detected obstacles coordinates x,y and rotation in z
                    self.detected_obstacles[id] = np.append(
                        self.coordinates[id][0:2], self.orientations[id][2])

                # Drawing aruco boxes and texts:
                top_left = [int(self.bbox[id][0, 0]) + 75,
                            int(self.bbox[id][0, 1])]
                aruco.drawAxis(
                    img, self.aruco_Cam_param[0], self.aruco_Cam_param[1], rvec[i], tvec[i], 60)
                cv2.putText(img, str(
                    f'x:{int(self.coordinates[id][0])} y: {int(self.coordinates[id][1])}'), top_left, cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)
                cv2.putText(img, str(f'x:{int(self.coordinates_estimated[id][0])} y: {int(self.coordinates_estimated[id][1])}'), [
                            top_left[0], top_left[1]-25], cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)

            return True

        return False

    # ------- save detected aruco as jsons files -------
    def save_Aruco_json(self):

        if self.ids is None:
            return

        obstacles_data = {"obstacles": []}
        robots_data = {"robots": []}
        area_data = {"area": []}

        # creating area json:
        for id in self.detected_corners.keys():

            area_data["area"].append(
                {
                    "id": int(id),
                    "position": {
                        "x": self.detected_corners[id][0],
                        "y": self.detected_corners[id][1]
                    },
                    "estimated_position": {
                        "x": self.coordinates_estimated[id][0],
                        "y": self.coordinates_estimated[id][1]

                    }
                }
            )

        # creating obstacles json:
        for id in self.detected_obstacles.keys():

            obstacles_data["obstacles"].append(
                {
                    "id": int(id),
                    "center": {
                        "x": self.detected_obstacles[id][0],
                        "y": self.detected_obstacles[id][1]
                    },
                    "rotation": self.detected_obstacles[id][2],
                    "estimated_position": {
                        "x": self.coordinates_estimated[id][0],
                        "y": self.coordinates_estimated[id][1]

                    }
                }
            )

        # creating robots json:
        for id in self.detected_robots.keys():

            robots_data["robots"].append(
                {
                    "id": int(id),
                    "position": {
                        "x": self.detected_robots[id][0],
                        "y": self.detected_robots[id][1]
                    },
                    "orientation": self.detected_robots[id][2],
                    "estimated_position": {
                        "x": self.coordinates_estimated[id][0],
                        "y": self.coordinates_estimated[id][1]

                    }
                }
            )

        # save data into as json files
        with open('./Computer/resources/obstacles.json', 'w') as f:
            json.dump(obstacles_data, f, indent=2)
            f.close()
        with open('./Computer/resources/robots.json', 'w') as f:
            json.dump(robots_data, f, indent=2)
            f.close()
        with open('./Computer/resources/area.json', 'w') as f:
            json.dump(area_data, f, indent=2)
            f.close()


# Camera class contain all setting of camera.
# Also allow to run as separate thread which hugly increase FPS.
class Camera():
    def __init__(self, camera_source=0, width=640, height=480, fps=40):
        ''' 
        :param camera_source: if 0 is not working try 1, 2, 3...
        :param width: choose for desired resolution 1920, 1280, 640
        :param height: choose for desired resolution 1080, 720, 480
        :param fps: choose desired FPS
        '''
        # initialize the video camera stream and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(camera_source)

        # setup camera (important for raspberry pi)
        # for raspberry pifor max FPS set 640x480
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_FPS, fps)

        # get calibration params from calibration file
        self.camera_calibration_params = self._get_calibration_params_()

        (self.grabbed, self.frame) = self.stream.read()
        # initialize the flag used to indicate if the thread should
        # be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

    def _get_calibration_params_(self):

        # load calibration values:
        with open('./Camera/camera_calibration.npy', 'rb') as f:
            camera_matrix = np.load(f)
            camera_distortion = np.load(f)

        return camera_matrix, camera_distortion

    def take_frame(self):
        return self.frame


# functions for matematical operations:
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles (x and z are swapped).
def rotationMatrixToEulerAngles(R):

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def main():

    aruco_markers = Aruco_markers()

    # start video capturing
    camera = Camera(0, resolution[0], resolution[1], FPS)
    aruco_markers.aruco_Cam_param = camera.camera_calibration_params
    # start separate thread:
    camera.start()

    # load images for augment
    aruco_markers.load_Aug_Images("./Camera/images")

    # start client for json sending:
    # enter IP of server:
    if args["json_sending"] > 0:
        client = RPI_Communication_Client(host=server_IP, port=server_PORT)

    # stop script conditions:
    print("\npress:\n- q to quit")

    def on_press(key):
        if key == keyboard.KeyCode.from_char('q'):
            return False
        return True

    # start listening for keyboard
    k = keyboard.Listener(on_press=on_press)
    k.start()

    while True:

        img = camera.take_frame()

        # find Aruco amrkers
        Is_aruco_Found = aruco_markers.find_Aruco_Markers(img)

        # save Aruco markers into json
        aruco_markers.save_Aruco_json()

        # loop through all the markers and augment each one
        if Is_aruco_Found:
            img = aruco_markers.augment_Aruco(img)

        if args["display"] > 0:
            cv2.imshow("ARUCO DETECTION", img)
            cv2.waitKey(1)

        # save stream video:
        cv2.imwrite(
            './Camera/stream_tests/flask_version/stream/stream.jpg', img)

        # send json files:
        if args["json_sending"] > 0:
            client.send_json(
                json.load(open("./Computer/resources/area.json", 'r')))
            client.send_json(
                json.load(open("./Computer/resources/obstacles.json", 'r')))
            client.send_json(
                json.load(open("./Computer/resources/robots.json", 'r')))

        # quit on pressed 'q':
        if not(k.is_alive()):
            print('\nquitting...\n')
            # stop video capture:
            camera.stop()
            # end main loop:
            break


if __name__ == '__main__':
    main()
