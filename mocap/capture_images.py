from pseyepy import Camera
import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
import time

c = Camera(fps=150, resolution=Camera.RES_SMALL, gain=10, exposure=100)
frame,_ = c.read()
frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
cv.imshow("ps3 eye", frame)

print("Press space to refresh image and '.' to capture image")
i = 0
while True:
    key = cv.waitKey(1)
    frame,_ = c.read()
    frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
    frame = np.rot90(frame, k=1)
    cv.imshow("ps3 eye", np.fliplr(frame))
    if key == ord(" "):
        frame,_ = c.read()
        frame,_ = c.read()
        frame,_ = c.read()
        frame = np.rot90(frame, k=3)
        frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
        cv.imwrite(f"{i}.jpg", frame)
        print(f'Image saved: {i}.jpg')
        i += 1
    elif key == ord("q"):
        exit()