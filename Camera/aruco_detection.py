from ctypes import sizeof
import cv2
from cv2 import aruco
from matplotlib.transforms import Bbox
import numpy as np
import os
import json
import math

# size of a side of aruco marker [mm]
marker_real_size = 35

def camera_setup(cap):
    
    # load calibration values:
    with open('./camera_calibration.npy', 'rb') as f:
        camera_matrix = np.load(f)
        camera_distortion = np.load(f)

    # setup camera (important for raspberry pi)
    # for raspberry pifor max FPS set 640x480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)       
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)      
    cap.set(cv2.CAP_PROP_FPS, 40)
    
    return  camera_matrix, camera_distortion


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
   
    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])


def load_Aug_Images(path):
    '''
    :param path: folder in which markers images are stored
    :return: dictionary with key as the id and values as the augment image
    '''
    myList = os.listdir(path)
    total_markers= len(myList)
    print(f'total number of imges: {total_markers}')
    aug_Dics = {}
    for imgPath in myList:
        key = int(os.path.splitext(imgPath)[0])
        imgAug = cv2.imread(f'{path}/{imgPath}')
        aug_Dics[key]= imgAug
    return aug_Dics
    

def find_Aruco_Markers(img, aruco_Param = None, marker_Size = 3, total_Markers=20, draw=True):
    '''
    :param img: img in which to find the aruco markers
    :if you use standard auruco markers check for right parameters
    :if you use 3x3 you can set your own
    :param marker_Size: defoult = 3
    :param total_Markers: defoult = 20
    :param draw: flag to draw bbox around markers detected
    :return: bounding boxes, id, coordinates and rotation of detected markers 
    '''
   
    img_Gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
   
    if (marker_Size<=3):
        aruco_Dict=aruco.custom_dictionary(total_Markers, marker_Size, 2020)
    else:
        key = getattr(aruco,f'DICT_{marker_Size}X{marker_Size}_{total_Markers}')
        aruco_Dict=aruco.Dictionary_get(key)

    bbox,ids,rejected = aruco.detectMarkers(img_Gray,aruco_Dict, aruco_Param)
    rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(bbox, marker_real_size, aruco_Param[0], aruco_Param[1])
   

    

    if ids is not None:
        markers_coordinates = np.zeros_like(tvec)
        markers_orientation = np.zeros_like(rvec)
        print(tvec[0][0])
        print(rvec[0][0])
        aruco. drawDetectedMarkers(img,bbox)
        for marker in range(len(ids)): 
            rotation_matrix, _ = cv2.Rodrigues(rvec[marker][0])
            markers_coordinates[marker][0] = np.dot(rotation_matrix, tvec[marker][0])
            markers_orientation[marker][0] = rotationMatrixToEulerAngles(rotation_matrix)

            top_left= int(bbox[marker][0][0][0]), int(bbox[marker][0][0][1])
            aruco.drawAxis(img, aruco_Param[0], aruco_Param[1], rvec[marker], tvec[marker], 100)    
            cv2.putText(img, str(f'x:{int(markers_coordinates[marker][0][0])} y: {int(markers_coordinates[marker][0][1])} z: {int(markers_coordinates[marker][0][2])} rot(z): {int(math.degrees(markers_orientation[marker][0][2]))}'), top_left, cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)
        
        return [bbox, ids, markers_coordinates, markers_orientation]  

    return [bbox, ids, None, None] 
   
   

def augment_Aruco(bbox, id, img, imgAug, drawId=True):
    '''
    :param bbox: the four corner points of the box
    :param id: markers id of the corresponding box used anly for display
    :param img: the final image on which to draw
    :param imgAug: the image that will be overlapped on the marker
    :param drawId: flag to display the id of the detected markers
    :return: image with the augment image overlaid
    '''

    top_left= int(bbox[0][0][0]), int(bbox[0][0][1])
    top_right= int(bbox[0][1][0]), int(bbox[0][1][1])
    bottom_right= int(bbox[0][2][0]), int(bbox[0][2][1])
    bottom_left= int(bbox[0][3][0]), int(bbox[0][3][1])

    h,w,c=imgAug.shape

    pts1 = np.array([top_left, top_right, bottom_right, bottom_left])
    pts2 = np.float32([[0,0],[w,0],[w,h],[0,h]])
    matrix, _ = cv2.findHomography(pts2,pts1)
    imgOut = cv2.warpPerspective(imgAug,matrix,(img.shape[1], img.shape[0]))
    cv2.fillConvexPoly(img, pts1.astype(int), (0,0,0))
    imgOut = img + imgOut

    if drawId:
        cv2.putText(imgOut, str(id), top_left, cv2.FONT_HERSHEY_PLAIN, 1, (255,0,255), 2)

    return imgOut

def save_Aruco_json(ids, markers_coordinats, markers_orientation):
    triangles_list_ids = [4, 5, 6]
    circles_list_ids = [7, 8, 9]
    rectangles_list_ids = [10, 11, 12]
    area_list_ids = [0, 1, 2, 3]
    robots_list_ids = [13, 14, 15]

    if ids is None:
        return

    obstacles_data = {"figures": []}
    robots_data = {"robots": []}
    area_data = {"area": []}

    for nr in range(len(ids)):
        if ids[nr] in triangles_list_ids:
            obstacles_data["figures"].append({
                "id":0,
                "type": "triangle",
                "point_1": {
                    "x": markers_coordinats[nr][0][0],
                    "y": markers_coordinats[nr][0][1]+10,
                },
                "point_2": {
                    "x": markers_coordinats[nr][0][0]+10,
                    "y": markers_coordinats[nr][0][1]-10,
                },
                "point_3": {
                    "x": markers_coordinats[nr][0][0]-10,
                    "y": markers_coordinats[nr][0][1]-10,
                }
                }
            )
        elif ids[nr] in circles_list_ids:
           obstacles_data["figures"].append(
                {
                "id":1,
                "type": "circle",
                "center": {
                    "x": markers_coordinats[nr][0][0],
                    "y": markers_coordinats[nr][0][1]
                },
                "radius": 200
                }
           )
        elif ids[nr] in rectangles_list_ids:
            obstacles_data["figures"].append({
                "id":2,
                "type": "rectangle",
                "center": {
                    "x": markers_coordinats[nr][0][0],
                    "y": markers_coordinats[nr][0][1]
                },
                "orientation": markers_orientation[nr][0][2],
                "length_x": 150,
                "length_y": 200
                })
        elif ids[nr] in robots_list_ids:
            robots_data["robots"].append({
                "id": int(ids[nr]),
                "type": "robot",
                "position": {
                    "x": markers_coordinats[nr][0][0],
                    "y": markers_coordinats[nr][0][1]
                },
                "orientation": markers_orientation[nr][0][2]
                }
            )
        elif ids[nr] in area_list_ids:
            area_data["area"].append({
                "id":0,
                "type": "corner",
                "position": {
                    "x": markers_coordinats[nr][0][0],
                    "y": markers_coordinats[nr][0][1]
                },
                "orientation": markers_orientation[nr][0][2]
                }
            )
        else:
            return


    # save data into files     
    with open('./TGS_obstackles.json', 'w') as f:
        json.dump(obstacles_data, f, indent=2)
        f.close()
    with open('./TGS_robots.json', 'w') as f:
        json.dump(robots_data, f, indent=2)
        f.close()
    with open('./TGS_area.json', 'w') as f:
        json.dump(area_data, f, indent=2)
        f.close()

def main():

    # video capture source camera 
    cap = cv2.VideoCapture(0) 
    aruco_Param = camera_setup(cap)


    # load images to augment  
    augDics = load_Aug_Images("images")


    print("press:\n\r- q to quit")

    while True:

        sccuess,img=cap.read()

        # find Aruco amrkers 
        aruco_Found = find_Aruco_Markers(img, aruco_Param)

        # save Aruco markers into json
        save_Aruco_json(aruco_Found[1], aruco_Found[2], aruco_Found[3])

        # loop through all the markers and augment each one
        if len(aruco_Found[0])!=0:
            for bbox, id in zip(aruco_Found[0], aruco_Found[1]):
                if int(id) in augDics.keys():
                    img= augment_Aruco(bbox, id, img, augDics[int(id)])
          

        cv2.imshow("ARUCO DETECTION",img)
          
        key = cv2.waitKey(1) & 0xFF
        # quit on 'q':
        if key ==ord('q'):
            break

        # ignore if any other key 
        else:
            pass

if __name__== '__main__':
    main()