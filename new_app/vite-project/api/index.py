from flask import Flask, Response, request
from pseyepy import Camera, Stream
import cv2 as cv
import numpy as np
import json
import os
from helpers import Singleton
from scipy import linalg, optimize
from flask_socketio import SocketIO, emit
from scipy.spatial.transform import Rotation

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
        self.camera_poses = None
    
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
                    object_point = triangulate_point(points, self.camera_poses)
                    error = calculate_reprojection_errors(np.array([points]), np.array([object_point]), self.camera_poses)[0]
                    socketio.emit("object-point", {"object_point": list(object_point), "error": error})
        
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

    def start_trangulating_points(self, camera_poses):
        self.is_capturing_points = True
        self.is_triangulating_points = True
        self.camera_poses = camera_poses

    def stop_trangulating_points(self):
        self.is_capturing_points = False
        self.is_triangulating_points = False
        self.camera_poses = None
    
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
    image_points = np.array(data["cameraPoints"])
    image_points_t = image_points.transpose((1, 0, 2))

    camera_poses = [{
        "R": np.eye(3),
        "t": np.array([[0],[0],[0]], dtype=np.float32)
    }]
    for camera_i in range(0, cameras.num_cameras-1):
        F, _ = cv.findFundamentalMat(image_points_t[camera_i], image_points_t[camera_i+1], cv.FM_RANSAC)
        E = cv.sfm.essentialFromFundamental(F, cameras.get_camera_params(0)["intrinsic_matrix"], cameras.get_camera_params(1)["intrinsic_matrix"])
        possible_Rs, possible_ts = cv.sfm.motionFromEssential(E)

        R = None
        t = None
        max_points_infront_of_camera = 0
        for i in range(0, 4):
            object_points = triangulate_points(image_points[:,camera_i:camera_i+2,:], np.concatenate([[camera_poses[-1]], [{"R": possible_Rs[i], "t": possible_ts[i]}]]))
            object_points_camera_coordinate_frame = np.array([possible_Rs[i].T @ object_point for object_point in object_points])

            points_infront_of_camera = np.sum(object_points[:,2] > 0) + np.sum(object_points_camera_coordinate_frame[:,2] > 0)

            if points_infront_of_camera > max_points_infront_of_camera:
                max_points_infront_of_camera = points_infront_of_camera
                R = possible_Rs[i]
                t = possible_ts[i]

        R = camera_poses[-1]["R"] @ R
        t = camera_poses[-1]["t"] + t

        camera_poses.append({
            "R": R,
            "t": t
        })

    camera_poses = bundle_adjustment(image_points, camera_poses)

    object_points = triangulate_points(image_points, camera_poses)
    error = np.mean(calculate_reprojection_errors(image_points, object_points, camera_poses))

    for i in range(0, len(camera_poses)):
        camera_poses[i] = {k: v.tolist() for (k, v) in camera_poses[i].items()}
    
    socketio.emit("camera-pose", {"error": error, "camera_poses": camera_poses})

def calculate_reprojection_errors(image_points, object_points, camera_poses):
    cameras = Cameras.instance()
    image_points_t = image_points.transpose((1, 0, 2))

    errors = np.array([])
    for i, camera_pose in enumerate(camera_poses):
        projected_img_points, _ = cv.projectPoints(object_points, np.array(camera_pose["R"], dtype=np.float64), np.array(camera_pose["t"], dtype=np.float64), cameras.get_camera_params(i)["intrinsic_matrix"], cameras.get_camera_params(i)["distortion_coef"])
        projected_img_points = projected_img_points[:,0,:]
        errors = np.concatenate([errors, (image_points_t[i]-projected_img_points).flatten() ** 2])

    return errors

def bundle_adjustment(image_points, camera_poses):
    def params_to_camera_poses(params):
        num_cameras = int(params.size/6)+1
        camera_poses = [{
            "R": np.eye(3),
            "t": np.array([0,0,0], dtype=np.float32)
        }]
        for i in range(0, num_cameras-1):
            camera_poses.append({
                "R": Rotation.as_matrix(Rotation.from_rotvec(params[i*6 : i*6 + 3])),
                "t": params[i*6 + 3 : i*6 + 6]
            })
        
        return camera_poses

    def residual_function(params):
        camera_poses = params_to_camera_poses(params)
        object_points = triangulate_points(image_points, camera_poses)
        errors = calculate_reprojection_errors(image_points, object_points, camera_poses)

        return errors

    init_params = np.array([])
    for camera_pose in camera_poses[1:]:
        rot_vec = Rotation.as_rotvec(Rotation.from_matrix(camera_pose["R"])).flatten()
        init_params = np.concatenate([init_params, rot_vec]) if init_params.size else rot_vec
        init_params = np.concatenate([init_params, camera_pose["t"].flatten()])

    res = optimize.least_squares(
        residual_function, init_params, verbose=2, ftol=1e-5
    )

    return params_to_camera_poses(res.x)
    
def triangulate_point(image_point, camera_poses):
    cameras = Cameras.instance()

    Ps = [] # projection matricies

    for i, camera_pose in enumerate(camera_poses):
        RT = np.c_[camera_pose["R"], camera_pose["t"]]
        P = cameras.camera_params[i]["intrinsic_matrix"] @ RT
        Ps.append(P)

    # https://temugeb.github.io/computer_vision/2021/02/06/direct-linear-transorms.html
    def DLT(Ps, image_point):
        A = []

        for P, image_point in zip(Ps, image_point):
            A.append(image_point[1]*P[2,:] - P[1,:])
            A.append(P[0,:] - image_point[0]*P[2,:])
            
        A = np.array(A).reshape((len(Ps)*2,4))
        B = A.transpose() @ A
        U, s, Vh = linalg.svd(B, full_matrices = False)
        object_point = Vh[3,0:3]/Vh[3,3]

        return object_point

    object_point = DLT(Ps, image_point)

    return object_point

def triangulate_points(image_points, camera_poses):
    object_points = []
    for image_point in image_points:
        object_point = triangulate_point(image_point, camera_poses)
        object_points.append(object_point)
    
    return np.array(object_points)

@socketio.on("triangulate-points")
def live_mocap(data):
    cameras = Cameras.instance()
    start_or_stop = data["startOrStop"]
    camera_poses = data["cameraPoses"]

    if (start_or_stop == "start"):
        cameras.start_trangulating_points(camera_poses)
        return
    elif (start_or_stop == "stop"):
        cameras.stop_trangulating_points()

        
if __name__ == '__main__':
    socketio.run(app, port=3001, debug=True)