from flask import Flask, Response, request
from pseyepy import Camera, Stream
import cv2 as cv
import numpy as np
import json
import os
from helpers import Singleton
from scipy import linalg
from flask_socketio import SocketIO, emit

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*')

@Singleton
class Cameras:
    def __init__(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, "camera-params.json")
        f = open(filename)
        self.camera_params = json.load(f)

        self.cameras = Camera(fps=150, resolution=Camera.RES_SMALL, gain=10, exposure=100)
        self.num_cameras = len(self.cameras.exposure)

        self.is_capturing_points = False

        self.is_triangulating_points = False
        self.r1 = None
        self.t1 = None
        self.r2 = None
        self.t2 = None
    
    def edit_settings(self, exposure, gain):
        self.cameras.exposure = [exposure] * self.num_cameras
        self.cameras.gain = [gain] * self.num_cameras

    def _camera_read(self):
        frames, _ = self.cameras.read()

        for i in range(0, self.num_cameras):
            frames[i] = np.rot90(frames[i], k=self.camera_params[i]["rotation"])
            frames[i] = cv.cvtColor(frames[i], cv.COLOR_RGB2BGR)

        if (self.is_capturing_points):
            points = []
            for i in range(0, self.num_cameras):
                frames[i], point = self._find_dot(frames[i])
                points.append(point)
            
            if (all([point[0]] is not None and point[1] is not None for point in points)):
                if self.is_capturing_points and not self.is_triangulating_points:
                    socketio.emit("image-points", points)
                elif self.is_triangulating_points:
                    object_point = triangulate_point(points, self.r1, self.t1, self.r2, self.t2)
                    socketio.emit("object-point", list(object_point))
        
        return frames

    def get_frames(self):
        frames = self._camera_read()

        return np.hstack(frames)

    def _find_dot(self, img):
        #img = cv.GaussianBlur(img,(5,5),0)
        grey = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        grey = cv.threshold(grey, 255*0.2, 255, cv.THRESH_BINARY)[1]
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

    def start_capturing_points(self):
        self.is_capturing_points = True

    def stop_capturing_points(self):
        self.is_capturing_points = False

    def start_trangulating_points(self, r1, t1, r2, t2):
        self.is_capturing_points = True
        self.is_triangulating_points = True
        self.r1 = r1
        self.t1 = t1
        self.r2 = r2
        self.t2 = t2

    def stop_trangulating_points(self):
        self.is_capturing_points = False
        self.is_triangulating_points = False
        self.r1 = None
        self.t1 = None
        self.r2 = None
        self.t2 = None
    
    def get_camera_params(self, camera_num):
        return {
            "intrinsic_matrix": np.array(self.camera_params[camera_num]["intrinsic_matrix"]),
            "distortion_coef": np.array(self.camera_params[camera_num]["distortion_coef"]),
            "rotation": self.camera_params[camera_num]["rotation"]
        }


@app.route("/api/camera-stream")
def camera_stream():
    cameras = Cameras.instance()

    def gen(cameras):
        while True:
            frames = cameras.get_frames()
            jpeg_frame = cv.imencode('.jpg', frames)[1].tostring()

            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + jpeg_frame + b'\r\n')

    return Response(gen(cameras), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on("update-camera-settings")
def change_camera_settings(data):
    cameras = Cameras.instance()
    
    cameras.edit_settings(data["exposure"], data["gain"])

@socketio.on("capture-points")
def capture_points(data):
    start_or_stop = data["startOrStop"]
    cameras = Cameras.instance()

    if (start_or_stop == "start"):
        cameras.start_capturing_points()
        return
    elif (start_or_stop == "stop"):
        cameras.stop_capturing_points()

@socketio.on("calculate-camera-pose")
def calculate_camera_pose(data):
    cameras = Cameras.instance()
    camera_points = np.array(data["cameraPoints"])

    pts1 = camera_points[:,0]
    pts2 = camera_points[:,1] 

    F, _ = cv.findFundamentalMat(pts1, pts2, cv.FM_RANSAC)
    print(F, flush=True)
    print(cameras.get_camera_params(0)["intrinsic_matrix"], flush=True)
    E = cv.sfm.essentialFromFundamental(F, cameras.get_camera_params(0)["intrinsic_matrix"], cameras.get_camera_params(1)["intrinsic_matrix"])
    possible_Rs, possible_ts = cv.sfm.motionFromEssential(E)

    R = None
    t = None
    max_points_infront_of_camera = 0
    for i in range(0, 4):
        object_points = triangulate_points(camera_points[:100], np.eye(3), np.zeros(3, dtype=np.float32), possible_Rs[i], possible_ts[i])
        object_points_camera_2_coordinate_frame = np.array([possible_Rs[i].T @ object_point for object_point in object_points])

        points_infront_of_camera = np.sum(object_points[:,2] > 0) + np.sum(object_points_camera_2_coordinate_frame[:,2] > 0)

        if points_infront_of_camera > max_points_infront_of_camera:
            max_points_infront_of_camera = points_infront_of_camera
            R = possible_Rs[i]
            t = possible_ts[i]

    res = {
        "R": R.tolist(),
        "t": t.tolist()
    }
    
    socketio.emit("camera-pose", res)
    
def triangulate_point(image_point, r1, t1, r2, t2):
    cameras = Cameras.instance()

    RT1 = np.c_[r1, t1]
    P1 = cameras.camera_params[0]["intrinsic_matrix"] @ RT1 #projection matrix for camera 1

    RT2 = np.c_[r2, t2]
    P2 = cameras.camera_params[1]["intrinsic_matrix"] @ RT2 #projection matrix for camera 2

    # https://temugeb.github.io/computer_vision/2021/02/06/direct-linear-transorms.html
    def DLT(P1, P2, point1, point2):

        A = [point1[1]*P1[2,:] - P1[1,:],
            P1[0,:] - point1[0]*P1[2,:],
            point2[1]*P2[2,:] - P2[1,:],
            P2[0,:] - point2[0]*P2[2,:]
            ]
        A = np.array(A).reshape((4,4))

        B = A.transpose() @ A
        U, s, Vh = linalg.svd(B, full_matrices = False)

        return Vh[3,0:3]/Vh[3,3]

    return DLT(P1, P2, image_point[0], image_point[1])

def triangulate_points(image_points, r1, t1, r2, t2):
    object_points = []
    for image_point in image_points:
        object_point = triangulate_point(image_point, r1, t1, r2, t2)
        object_points.append(object_point)
    
    return np.array(object_points)

@socketio.on("triangulate-points")
def live_mocap(data):
    cameras = Cameras.instance()
    start_or_stop = data["startOrStop"]
    r1 = data["r1"]
    t1 = data["t1"]
    r2 = data["r2"]
    t2 = data["t2"]

    if (start_or_stop == "start"):
        cameras.start_trangulating_points(r1, t1, r2, t2)
        return
    elif (start_or_stop == "stop"):
        cameras.stop_trangulating_points()

        
if __name__ == '__main__':
    socketio.run(app, port=3001, debug=True)