from pseyepy import Camera
import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv

c = Camera(fps=150, resolution=Camera.RES_SMALL, gain=0, exposure=100)

camera_params = [
    {
        "intrinsic_matrix": np.array([[268.66976067,   0.,123.58679484],
                            [  0.,         268.57495496, 167.56126939],
                            [  0.,           0.,           1.        ]]),    
                             "distortion_coef": np.array([-1.26372388e-01,  2.62661497e-01,  1.21306197e-03,  2.24507008e-04,  -2.48534118e-01]),
        "rotation": 1
    },
    {
        "intrinsic_matrix": np.array([[269.95158059,   0, 139.37072352],
                             [  0,          270.09608831, 160.36482761],
                             [  0,            0,            1,        ]]),        
                             "distortion_coef": np.array([-1.17985085e-01,  1.73234197e-01,  1.48276201e-03,  1.31644936e-04, -9.25910705e-02]),
        "rotation": 3
    },
]

def init_frame(img, camera_num): 
    h, w = img.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(
        np.array(camera_params[camera_num]["intrinsic_matrix"]), 
        np.array(camera_params[camera_num]["distortion_coef"]), (w, h), 1, (w, h)
    )
    img = cv.undistort(img, np.array(camera_params[camera_num]["intrinsic_matrix"]), np.array(camera_params[camera_num]["distortion_coef"]), None, newcameramtx)
    x, y, w, h = roi
    img = img[y:y+h, x:x+w]
    img = np.rot90(img, k=camera_params[camera_num]["rotation"])
    img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
   
    return img

def find_dot(img):
    img = cv.GaussianBlur(img,(5,5),0)
    grey = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    grey = cv.threshold(grey, 255*0.4, 255, cv.THRESH_BINARY)[1]
    contours,_ = cv.findContours(grey, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    img = cv.drawContours(img, contours, -1, (0,255,0), 1)

    center_x, center_y = None, None
    if len(contours) != 0:
        moments = cv.moments(contours[0])
        if moments["m00"] != 0:
            center_x = int(moments["m10"] / moments["m00"])
            center_y = int(moments["m01"] / moments["m00"])
            cv.putText(img, f'({center_x}, {center_y})', (center_x,center_y - 15), cv.FONT_HERSHEY_SIMPLEX, 0.3, (100,255,100), 1)
            cv.circle(img, (center_x,center_y), 1, (100,255,100), -1)


    return img, [center_x, center_y]

while True:
    key = cv.waitKey(1)
    imgs,_ = c.read()
    imgs = [init_frame(img, i) for i, img in enumerate(imgs)]
    img1, dot1 = find_dot(imgs[0])
    cv.imshow("ps3 eye 1", img1)
    img2, dot2 = find_dot(imgs[1])
    cv.imshow("ps3 eye 2", img2)
    if key == ord(" "):
        if (dot1[0] and dot1[1] and dot2[0] and dot2[1]):
            print(f"[{dot1}, {dot2}],", end="")
    # if (dot1[0] and dot1[1] and dot2[0] and dot2[1]):
    #     print(f"[{dot1}, {dot2}],", end="")
    elif key == ord("q"):
        exit()