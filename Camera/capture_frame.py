
import cv2
import time
import numpy as np

cv2.namedWindow

# video capture source camera
cap = cv2.VideoCapture(0)

# setup camera (iportant for raspberry pi)
# for raspberry pifor max FPS set 640x480
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 40)

prev_frame_time = time.time()

print("press:\n\r- q to quit\n\r- s to save frame")

img_count = 0  # for counting captured frames

while(True):

    # return a single frame in variable `frame`
    ret, frame = cap.read()

    # add FPS value on the picture
    new_frame_time = time.time()
    fps = 1/(new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time
    cv2.putText(frame, f"FPS {int(fps)}", (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 250, 0), cv2.LINE_4)

    # display the captured image
    cv2.imshow('IMAGE', frame)

    # save on pressing 's'
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        cv2.imwrite(
            f'./Camera/captured_frames/calibration_img_{img_count}.png', frame)
        img_count += 1
        print(f"saved frames: {img_count}")

    # quit on 'q':
    elif key == ord('q'):
        break

    # ignore if any other key
    else:
        pass

cap.release()
cv2.destroyAllWindows()
