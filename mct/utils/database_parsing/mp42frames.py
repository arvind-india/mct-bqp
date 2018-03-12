# Basically a glorified ffmpeg wrapper

import cv2

vidcap = cv2.VideoCapture('~/mct-bqp/ucla_data/parkinglot/view-GL6.mp4')
success,image = vidcap.read()
count = 0

success = True

while success:
    success,image = vidcap.read()
    print('Read a new frame: ', success)
    cv2.imwrite("~/mct-bqp/ucla_data/parkinglot/frame%d.jpg" % count, image)     # save frame as JPEG file
    count += 1
