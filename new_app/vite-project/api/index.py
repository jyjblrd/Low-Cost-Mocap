from flask import Flask, Response, request
from pseyepy import Camera, Stream
import math
import cv2 as cv
import numpy as np
import json
import os
from helpers import Singleton
from scipy import linalg, optimize, signal

from flask_socketio import SocketIO, emit
from scipy.spatial.transform import Rotation
from scipy.signal import butter, lfilter
import copy
import time
from sklearn.preprocessing import normalize
import serial
import threading

serialLock = threading.Lock()

ser = serial.Serial("/dev/cu.usbserial-02X2K2GE", 460800, write_timeout=1, )

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*')

cameras_init = False

class LowPassFilter:

    def __init__(self, cutoff_frequency, sampling_frequency, dims, order=5, buffer_size=300):
        self.sampling_frequency = sampling_frequency
        self.cutoff_frequency = cutoff_frequency
        self.order = order
        self.buffer_size = buffer_size
        self.buffered_data = np.empty((0, dims))
        self.b, self.a = butter(self.order, self.cutoff_frequency / (self.sampling_frequency / 2), btype='low')

    def filter(self, data):
        self.buffered_data = np.vstack((self.buffered_data, data[np.newaxis]))

        filtered_data = np.apply_along_axis(lambda x: lfilter(self.b, self.a, x), axis=0, arr=self.buffered_data)
    
        if self.buffered_data.shape[0] >= self.buffer_size:
            self.buffered_data = self.buffered_data[-self.buffer_size//2:]  # Keep the last half of the buffered data for the next filtering iteration

        return filtered_data[-1]


low_pass_filter = LowPassFilter(cutoff_frequency=10, sampling_frequency=60.0, dims=3)
heading_low_pass_filter = LowPassFilter(cutoff_frequency=3, sampling_frequency=60.0, dims=1)

class KalmanFilter:
    def __init__(self):
        state_dim = 9
        measurement_dim = 6
        dt = 0.1

        self.kalman = cv.KalmanFilter(state_dim, measurement_dim)
        self.kalman.transitionMatrix = np.array([[1, 0, 0, dt, 0, 0, 0.5*dt**2, 0, 0],
                                                 [0, 1, 0, 0, dt, 0, 0, 0.5*dt**2, 0],
                                                 [0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt**2],
                                                 [0, 0, 0, 1, 0, 0, dt, 0, 0],
                                                 [0, 0, 0, 0, 1, 0, 0, dt, 0],
                                                 [0, 0, 0, 0, 0, 1, 0, 0, dt],
                                                 [0, 0, 0, 0, 0, 0, 1, 0, 0],
                                                 [0, 0, 0, 0, 0, 0, 0, 1, 0],
                                                 [0, 0, 0, 0, 0, 0, 0, 0, 1]], dtype=np.float32)

        self.kalman.processNoiseCov = np.eye(state_dim, dtype=np.float32) * 1e-2
        self.kalman.measurementNoiseCov = np.eye(measurement_dim, dtype=np.float32) * 1e0
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                                                  [0, 1, 0, 0, 0, 0, 0, 0, 0],
                                                  [0, 0, 1, 0, 0, 0, 0, 0, 0],
                                                  [0, 0, 0, 1, 0, 0, 0, 0, 0],
                                                  [0, 0, 0, 0, 1, 0, 0, 0, 0],
                                                  [0, 0, 0, 0, 0, 1, 0, 0, 0]], dtype=np.float32)
        
        self.kalman.statePost = np.zeros((9,1), dtype=np.float32)
        
        self.prev_measurement_time = 0
        self.prev_pos = np.array([0,0,0])

    def predict_location(self, possible_new_objects):
        if len(possible_new_objects) == 0:
            return {
                "pos": np.array([]),
                "vel": np.array([]),
                "heading": 0
            }
        
        possible_new_positions = [x["pos"] for x in possible_new_objects]

        dt = time.time() - self.prev_measurement_time
        self.prev_measurement_time = time.time()

        self.kalman.transitionMatrix[:3, 3:6] = dt * np.eye(3)
        self.kalman.transitionMatrix[3:6, 6:9] = dt * np.eye(3)
        self.kalman.transitionMatrix[:3, 6:9] = 0.5 * dt**2 * np.eye(3)

        predicted_location = self.kalman.predict()[:3].T[0]
        distances_to_predicted_location = np.sqrt(np.sum((possible_new_positions - predicted_location)**2, axis=1))
        closest_match_i = np.argmin(distances_to_predicted_location)
        new_pos = possible_new_positions[closest_match_i].astype(np.float32)
        new_vel = ((new_pos - self.prev_pos) / dt).astype(np.float32)
        self.prev_pos = new_pos

        self.kalman.correct(np.concatenate((new_pos, new_vel)))
        predicted_state = self.kalman.statePre[:6].T[0]  # Predicted 3D location

        heading = possible_new_objects[closest_match_i]["heading"]
        heading = heading_low_pass_filter.filter(heading)[0]
        # heading, self.z = signal.lfilter(self.b, 1, [heading], zi=self.z)

        vel = predicted_state[3:6].copy()
        vel = low_pass_filter.filter(vel)
        
        return {
            "pos": predicted_state[:3],
            "vel": vel,
            "heading": heading
        }
    
kalman_filter = KalmanFilter()

@Singleton
class Cameras:
    def __init__(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, "camera-params.json")
        f = open(filename)
        self.camera_params = json.load(f)

        self.cameras = Camera(fps=90, resolution=Camera.RES_SMALL, gain=10, exposure=100)
        self.num_cameras = len(self.cameras.exposure)
        print(self.num_cameras)

        self.is_capturing_points = False

        self.is_triangulating_points = False
        self.camera_poses = None

        self.is_locating_objects = False

        self.to_world_coords_matrix = None

        self.drone_armed = False

        global cameras_init
        cameras_init = True
    
    def edit_settings(self, exposure, gain):
        self.cameras.exposure = [exposure] * self.num_cameras
        self.cameras.gain = [gain] * self.num_cameras

    def _camera_read(self):
        frames, _ = self.cameras.read()

        for i in range(0, self.num_cameras):
            frames[i] = np.rot90(frames[i], k=self.camera_params[i]["rotation"])
            frames[i] = make_square(frames[i])
            frames[i] = cv.undistort(frames[i], self.get_camera_params(i)["intrinsic_matrix"], self.get_camera_params(i)["distortion_coef"])
            frames[i] = cv.GaussianBlur(frames[i],(9,9),0)
            kernel = np.array([[-2,-1,-1,-1,-2],
                               [-1,1,3,1,-1],
                               [-1,3,4,3,-1],
                               [-1,1,3,1,-1],
                               [-2,-1,-1,-1,-2]])
            frames[i] = cv.filter2D(frames[i], -1, kernel)
            frames[i] = cv.cvtColor(frames[i], cv.COLOR_RGB2BGR)

        if (self.is_capturing_points):
            image_points = []
            for i in range(0, self.num_cameras):
                frames[i], single_camera_image_points = self._find_dot(frames[i])
                image_points.append(single_camera_image_points)
            
            if (any(np.all(point[0] != [None,None]) for point in image_points)):
                if self.is_capturing_points and not self.is_triangulating_points:
                    socketio.emit("image-points", [x[0] for x in image_points])
                elif self.is_triangulating_points:
                    errors, object_points, frames = find_point_correspondance_and_object_points(image_points, self.camera_poses, frames)

                    # convert to world coordinates
                    for i, object_point in enumerate(object_points):
                        new_object_point = np.array([[-1,0,0],[0,-1,0],[0,0,1]]) @ object_point
                        new_object_point = np.concatenate((new_object_point, [1]))
                        new_object_point = np.array(self.to_world_coords_matrix) @ new_object_point
                        new_object_point = new_object_point[:3] / new_object_point[3]
                        new_object_point[1], new_object_point[2] = new_object_point[2], new_object_point[1]
                        object_points[i] = new_object_point

                    objects = []
                    filtered_object = {
                        "pos": [],
                        "vel": [],
                        "heading": 0
                    }
                    if self.is_locating_objects:
                        objects = locate_objects(object_points, errors)
                        filtered_object = kalman_filter.predict_location(objects)

                        if len(filtered_object["pos"]) != 0:
                            if self.drone_armed:
                                filtered_object["heading"] = round(filtered_object["heading"], 4)

                                serial_data = { 
                                    "pos": [round(x, 4) for x in filtered_object["pos"].tolist()] + [filtered_object["heading"]],
                                    "vel": [round(x, 4) for x in filtered_object["vel"].tolist()]
                                }
                                print(filtered_object)
                                with serialLock:
                                    ser.write(json.dumps(serial_data).encode('utf-8'))
                            
                        filtered_object["vel"] = filtered_object["vel"].tolist()
                        filtered_object["pos"] = filtered_object["pos"].tolist()
                    
                    socketio.emit("object-points", {
                        "object_points": object_points.tolist(), 
                        "errors": errors.tolist(), 
                        "objects": [{k:v.tolist() for (k,v) in object.items()} for object in objects], 
                        "filtered_object": [filtered_object]
                    })
        
        return frames

    def get_frames(self):
        frames = self._camera_read()

        return np.hstack(frames)

    def _find_dot(self, img):
        # img = cv.GaussianBlur(img,(5,5),0)
        grey = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        grey = cv.threshold(grey, 255*0.2, 255, cv.THRESH_BINARY)[1]
        contours,_ = cv.findContours(grey, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        img = cv.drawContours(img, contours, -1, (0,255,0), 1)

        image_points = []
        for contour in contours:
            moments = cv.moments(contour)
            if moments["m00"] != 0:
                center_x = int(moments["m10"] / moments["m00"])
                center_y = int(moments["m01"] / moments["m00"])
                cv.putText(img, f'({center_x}, {center_y})', (center_x,center_y - 15), cv.FONT_HERSHEY_SIMPLEX, 0.3, (100,255,100), 1)
                cv.circle(img, (center_x,center_y), 1, (100,255,100), -1)
                image_points.append([center_x, center_y])

        if len(image_points) == 0:
            image_points = [[None, None]]

        return img, image_points

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

    def start_locating_objects(self):
        self.is_locating_objects = True

    def stop_locating_objects(self):
        self.is_locating_objects = False
    
    def get_camera_params(self, camera_num):
        return {
            "intrinsic_matrix": np.array(self.camera_params[camera_num]["intrinsic_matrix"]),
            "distortion_coef": np.array(self.camera_params[camera_num]["distortion_coef"]),
            "rotation": self.camera_params[camera_num]["rotation"]
        }
    
    def set_camera_params(self, camera_num, intrinsic_matrix=None, distortion_coef=None):
        if intrinsic_matrix is not None:
            self.camera_params[camera_num]["intrinsic_matrix"] = intrinsic_matrix
        
        if distortion_coef is not None:
            self.camera_params[camera_num]["distortion_coef"] = distortion_coef


@app.route("/api/camera-stream")
def camera_stream():
    cameras = Cameras.instance()
    

    def gen(cameras):
        frequency = 150
        loop_interval = 1.0 / frequency
        last_run_time = 0

        while True:
            time_now = time.time()
            if time_now - last_run_time < loop_interval:
                time.sleep(last_run_time - time_now + loop_interval)
            last_run_time = time.time()
            frames = cameras.get_frames()
            jpeg_frame = cv.imencode('.jpg', frames)[1].tostring()

            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + jpeg_frame + b'\r\n')

    return Response(gen(cameras), mimetype='multipart/x-mixed-replace; boundary=frame')


@socketio.on("arm-drone")
def arm_drone(data):
    global cameras_init
    if not cameras_init:
        return
    
    Cameras.instance().drone_armed = data["droneArmed"]
    serial_data = {
        "armed": data["droneArmed"],
    }
    with serialLock:
        print(data["count"])
        ser.write(json.dumps(serial_data).encode('utf-8'))

@socketio.on("set-drone-pid")
def arm_drone(data):
    serial_data = {
        "pid": [float(x) for x in data["dronePID"]],
    }
    with serialLock:
        ser.write(json.dumps(serial_data).encode('utf-8'))

@socketio.on("set-drone-setpoint")
def arm_drone(data):
    serial_data = {
        "setpoint": [float(x) for x in data["droneSetpoint"]],
    }
    with serialLock:
        ser.write(json.dumps(serial_data).encode('utf-8'))

@socketio.on("set-drone-trim")
def arm_drone(data):
    serial_data = {
        "trim": [int(x) for x in data["droneTrim"]],
    }
    with serialLock:
        ser.write(json.dumps(serial_data).encode('utf-8'))


@socketio.on("acquire-floor")
def acquire_floor(data):
    cameras = Cameras.instance()
    object_points = data["objectPoints"]
    object_points = np.array([item for sublist in object_points for item in sublist])

    tmp_A = []
    tmp_b = []
    for i in range(len(object_points)):
        tmp_A.append([object_points[i,0], object_points[i,1], 1])
        tmp_b.append(object_points[i,2])
    b = np.matrix(tmp_b).T
    A = np.matrix(tmp_A)

    fit, residual, rnk, s = linalg.lstsq(A, b)
    fit = fit.T[0]

    plane_normal = np.array([[fit[0]], [fit[1]], [-1]])
    plane_normal = plane_normal / linalg.norm(plane_normal)
    up_normal = np.array([[0],[0],[1]], dtype=np.float32)

    plane = np.array([fit[0], fit[1], -1, fit[2]])

    # https://math.stackexchange.com/a/897677/1012327
    G = np.array([
        [np.dot(plane_normal.T,up_normal)[0][0], -linalg.norm(np.cross(plane_normal.T[0],up_normal.T[0])), 0],
        [linalg.norm(np.cross(plane_normal.T[0],up_normal.T[0])), np.dot(plane_normal.T,up_normal)[0][0], 0],
        [0, 0, 1]
    ])
    F = np.array([plane_normal.T[0], ((up_normal-np.dot(plane_normal.T,up_normal)[0][0]*plane_normal)/linalg.norm((up_normal-np.dot(plane_normal.T,up_normal)[0][0]*plane_normal))).T[0], np.cross(up_normal.T[0],plane_normal.T[0])]).T
    R = F @ G @ linalg.inv(F)

    R = R @ [[1,0,0],[0,-1,0],[0,0,1]] # i dont fucking know why

    cameras.to_world_coords_matrix = np.array(np.vstack((np.c_[R, [0,0,0]], [[0,0,0,1]])))

    socketio.emit("to-world-coords-matrix", {"to_world_coords_matrix": cameras.to_world_coords_matrix.tolist()})


@socketio.on("set-origin")
def set_origin(data):
    cameras = Cameras.instance()
    object_point = np.array(data["objectPoint"])
    to_world_coords_matrix = np.array(data["toWorldCoordsMatrix"])
    transform_matrix = np.eye(4)

    object_point[1], object_point[2] = object_point[2], object_point[1] # i dont fucking know why
    transform_matrix[:3, 3] = -object_point

    to_world_coords_matrix = transform_matrix @ to_world_coords_matrix
    cameras.to_world_coords_matrix = to_world_coords_matrix

    socketio.emit("to-world-coords-matrix", {"to_world_coords_matrix": cameras.to_world_coords_matrix.tolist()})

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
        camera1_image_points = image_points_t[camera_i]
        camera2_image_points = image_points_t[camera_i+1]
        not_none_indicies = np.where(np.all(camera1_image_points != None, axis=1) & np.all(camera2_image_points != None, axis=1))[0]
        camera1_image_points = np.take(camera1_image_points, not_none_indicies, axis=0).astype(np.float32)
        camera2_image_points = np.take(camera2_image_points, not_none_indicies, axis=0).astype(np.float32)

        F, _ = cv.findFundamentalMat(camera1_image_points, camera2_image_points, cv.FM_RANSAC, 1, 0.99999)
        E = cv.sfm.essentialFromFundamental(F, cameras.get_camera_params(0)["intrinsic_matrix"], cameras.get_camera_params(1)["intrinsic_matrix"])
        possible_Rs, possible_ts = cv.sfm.motionFromEssential(E)

        R = None
        t = None
        max_points_infront_of_camera = 0
        for i in range(0, 4):
            object_points = triangulate_points(np.hstack([np.expand_dims(camera1_image_points, axis=1), np.expand_dims(camera2_image_points, axis=1)]), np.concatenate([[camera_poses[-1]], [{"R": possible_Rs[i], "t": possible_ts[i]}]]))
            object_points_camera_coordinate_frame = np.array([possible_Rs[i].T @ object_point for object_point in object_points])

            points_infront_of_camera = np.sum(object_points[:,2] > 0) + np.sum(object_points_camera_coordinate_frame[:,2] > 0)

            if points_infront_of_camera > max_points_infront_of_camera:
                max_points_infront_of_camera = points_infront_of_camera
                R = possible_Rs[i]
                t = possible_ts[i]

        R = R @ camera_poses[-1]["R"]
        t = camera_poses[-1]["t"] + (camera_poses[-1]["R"] @ t)

        camera_poses.append({
            "R": R,
            "t": t
        })

    camera_poses = bundle_adjustment(image_points, camera_poses)

    object_points = triangulate_points(image_points, camera_poses)
    error = np.mean(calculate_reprojection_errors(image_points, object_points, camera_poses))

    socketio.emit("camera-pose", {"camera_poses": camera_pose_to_serializable(camera_poses)})

def calculate_reprojection_errors(image_points, object_points, camera_poses):
    errors = np.array([])
    for image_points_i, object_point in zip(image_points, object_points):
        error = calculate_reprojection_error(image_points_i, object_point, camera_poses)
        if error is None:
            continue
        errors = np.concatenate([errors, [error]])

    return errors

def calculate_reprojection_error(image_points, object_point, camera_poses):
    cameras = Cameras.instance()

    image_points = np.array(image_points)
    none_indicies = np.where(np.all(image_points == None, axis=1))[0]
    image_points = np.delete(image_points, none_indicies, axis=0)
    camera_poses = np.delete(camera_poses, none_indicies, axis=0)

    if len(image_points) <= 1:
        return None

    image_points_t = image_points.transpose((0,1))

    errors = np.array([])
    for i, camera_pose in enumerate(camera_poses):
        if np.all(image_points[i] == None, axis=0):
            continue
        projected_img_points, _ = cv.projectPoints(
            np.expand_dims(object_point, axis=0).astype(np.float32), 
            np.array(camera_pose["R"], dtype=np.float64), 
            np.array(camera_pose["t"], dtype=np.float64), 
            cameras.get_camera_params(i)["intrinsic_matrix"], 
            np.array([])
        )
        projected_img_point = projected_img_points[:,0,:][0]
        errors = np.concatenate([errors, (image_points_t[i]-projected_img_point).flatten() ** 2])
    
    return errors.mean()


def bundle_adjustment(image_points, camera_poses):
    cameras = Cameras.instance()

    def params_to_camera_poses(params):
        focal_distances = []
        num_cameras = int((params.size-1)/7)+1
        camera_poses = [{
            "R": np.eye(3),
            "t": np.array([0,0,0], dtype=np.float32)
        }]
        focal_distances.append(params[0])
        for i in range(0, num_cameras-1):
            focal_distances.append(params[i*7+1])
            camera_poses.append({
                "R": Rotation.as_matrix(Rotation.from_rotvec(params[i*7 + 2 : i*7 + 3 + 2])),
                "t": params[i*7 + 3 + 2 : i*7 + 6 + 2]
            })

        return camera_poses, focal_distances

    def residual_function(params):
        camera_poses, focal_distances = params_to_camera_poses(params)
        for i in range(0, len(camera_poses)):
            intrinsic = cameras.get_camera_params(i)["intrinsic_matrix"]
            intrinsic[0, 0] = focal_distances[i]
            intrinsic[1, 1] = focal_distances[i]
            # cameras.set_camera_params(i, intrinsic)
        object_points = triangulate_points(image_points, camera_poses)
        errors = calculate_reprojection_errors(image_points, object_points, camera_poses)
        errors = errors.astype(np.float32)
        socketio.emit("camera-pose", {"camera_poses": camera_pose_to_serializable(camera_poses)})
        
        return errors

    focal_distance = cameras.get_camera_params(0)["intrinsic_matrix"][0,0]
    init_params = np.array([focal_distance])
    for i, camera_pose in enumerate(camera_poses[1:]):
        rot_vec = Rotation.as_rotvec(Rotation.from_matrix(camera_pose["R"])).flatten()
        focal_distance = cameras.get_camera_params(i)["intrinsic_matrix"][0,0]
        init_params = np.concatenate([init_params, [focal_distance]])
        init_params = np.concatenate([init_params, rot_vec])
        init_params = np.concatenate([init_params, camera_pose["t"].flatten()])

    res = optimize.least_squares(
        residual_function, init_params, verbose=2, ftol=1e-2
    )

    print(res.x)
    return params_to_camera_poses(res.x)[0]
    
def triangulate_point(image_points, camera_poses):
    image_points = np.array(image_points)
    cameras = Cameras.instance()
    none_indicies = np.where(np.all(image_points == None, axis=1))[0]
    image_points = np.delete(image_points, none_indicies, axis=0)
    camera_poses = np.delete(camera_poses, none_indicies, axis=0)

    if len(image_points) <= 1:
        return [None, None, None]

    Ps = [] # projection matricies

    for i, camera_pose in enumerate(camera_poses):
        RT = np.c_[camera_pose["R"], camera_pose["t"]]
        P = cameras.camera_params[i]["intrinsic_matrix"] @ RT
        Ps.append(P)

    # https://temugeb.github.io/computer_vision/2021/02/06/direct-linear-transorms.html
    def DLT(Ps, image_points):
        A = []

        for P, image_point in zip(Ps, image_points):
            A.append(image_point[1]*P[2,:] - P[1,:])
            A.append(P[0,:] - image_point[0]*P[2,:])
            
        A = np.array(A).reshape((len(Ps)*2,4))
        B = A.transpose() @ A
        U, s, Vh = linalg.svd(B, full_matrices = False)
        object_point = Vh[3,0:3]/Vh[3,3]

        return object_point

    object_point = DLT(Ps, image_points)

    return object_point

def triangulate_points(image_points, camera_poses):
    object_points = []
    for image_points_i in image_points:
        object_point = triangulate_point(image_points_i, camera_poses)
        object_points.append(object_point)
    
    return np.array(object_points)

def find_point_correspondance_and_object_points(image_points, camera_poses, frames):
    cameras = Cameras.instance()

    for image_points_i in image_points:
        try:
            image_points_i.remove([None, None])
        except:
            pass

    # [object_points, possible image_point groups, image_point from camera]
    correspondances = [[[i]] for i in image_points[0]]

    Ps = [] # projection matricies
    for i, camera_pose in enumerate(camera_poses):
        RT = np.c_[camera_pose["R"], camera_pose["t"]]
        P = cameras.camera_params[i]["intrinsic_matrix"] @ RT
        Ps.append(P)

    root_image_points = [{"camera": 0, "point": point} for point in image_points[0]]

    for i in range(1, len(camera_poses)):
        epipolar_lines = []
        for root_image_point in root_image_points:
            F = cv.sfm.fundamentalFromProjections(Ps[root_image_point["camera"]], Ps[i])
            line = cv.computeCorrespondEpilines(np.array([root_image_point["point"]], dtype=np.float32), 1, F)
            epipolar_lines.append(line[0,0].tolist())
            frames[i] = drawlines(frames[i], line[0])

        unmatched_image_points = np.array(image_points[i])
        points = np.array(image_points[i])

        for j, [a, b, c] in enumerate(epipolar_lines):
            distances_to_line = np.array([])
            if len(points) != 0:
                distances_to_line = np.abs(a*points[:,0] + b*points[:,1] + c) / np.sqrt(a**2 + b**2)
            possible_matches = points[distances_to_line < 20]
    
            if len(possible_matches) == 0:
                for possible_group in correspondances[j]:
                    possible_group.append([None, None])
            else:
                unmatched_image_points = [row for row in unmatched_image_points.tolist() if row not in possible_matches.tolist()]
                unmatched_image_points = np.array(unmatched_image_points)
                new_correspondances_j = []
                for possible_match in possible_matches:
                    temp = copy.deepcopy(correspondances[j])
                    for possible_group in temp:
                        possible_group.append(possible_match.tolist())
                    new_correspondances_j += temp
                correspondances[j] = new_correspondances_j

        for unmatched_image_point in unmatched_image_points:
            root_image_points.append({"camera": i, "point": unmatched_image_point})
            temp = [[[None, None]] * i]
            temp[0].append(unmatched_image_point.tolist())
            correspondances.append(temp)

    object_points = []
    errors = []
    for image_points in correspondances:
        object_points_i = triangulate_points(image_points, camera_poses)

        if np.all(object_points_i == None):
            continue

        errors_i = calculate_reprojection_errors(image_points, object_points_i, camera_poses)

        object_points.append(object_points_i[np.argmin(errors_i)])
        errors.append(np.min(errors_i))

    return np.array(errors), np.array(object_points), frames

def locate_objects(object_points, errors):
    dist = 0.145

    distance_matrix = np.zeros((object_points.shape[0], object_points.shape[0]))
    already_matched_points = []
    objects = []

    for i in range(0, object_points.shape[0]):
        for j in range(0, object_points.shape[0]):
            distance_matrix[i,j] = np.sqrt(np.sum((object_points[i] - object_points[j])**2))

    for i in range(0, object_points.shape[0]):
        if i in already_matched_points:
            continue

        matches = np.abs(distance_matrix[i] - dist) < 0.025
        if np.any(matches):
            best_match_i = np.argmax(matches)

            already_matched_points.append(i)
            already_matched_points.append(best_match_i)

            location = (object_points[i]+object_points[best_match_i])/2
            error = np.mean([errors[i], errors[best_match_i]])

            heading_vec = object_points[best_match_i] - object_points[i]
            heading_vec /= linalg.norm(heading_vec)
            heading = np.arctan2(heading_vec[1], heading_vec[0])

            heading = heading - np.pi if heading > np.pi/2 else heading
            heading = heading + np.pi if heading < -np.pi/2 else heading

            objects.append({
                "pos": location,
                "heading": -heading,
                "error": error
            })
    
    return objects

@socketio.on("locate-objects")
def start_or_stop_locating_objects(data):
    cameras = Cameras.instance()
    start_or_stop = data["startOrStop"]

    if (start_or_stop == "start"):
        cameras.start_locating_objects()
        return
    elif (start_or_stop == "stop"):
        cameras.stop_locating_objects()

@socketio.on("determine-scale")
def determine_scale(data):
    object_points = data["objectPoints"]
    camera_poses = data["cameraPoses"]
    actual_distance = 0.1
    observed_distances = []

    for object_points_i in object_points:
        if len(object_points_i) != 2:
            continue

        object_points_i = np.array(object_points_i)

        observed_distances.append(np.sqrt(np.sum((object_points_i[0] - object_points_i[1])**2)))

    scale_factor = actual_distance/np.mean(observed_distances)
    for i in range(0, len(camera_poses)):
        camera_poses[i]["t"] = (np.array(camera_poses[i]["t"]) * scale_factor).tolist()

    socketio.emit("camera-pose", {"error": None, "camera_poses": camera_poses})


@socketio.on("triangulate-points")
def live_mocap(data):
    cameras = Cameras.instance()
    start_or_stop = data["startOrStop"]
    camera_poses = data["cameraPoses"]
    cameras.to_world_coords_matrix = data["toWorldCoordsMatrix"]

    if (start_or_stop == "start"):
        cameras.start_trangulating_points(camera_poses)
        return
    elif (start_or_stop == "stop"):
        cameras.stop_trangulating_points()

def numpy_fillna(data):
    data = np.array(data, dtype=object)
    # Get lengths of each row of data
    lens = np.array([len(i) for i in data])

    # Mask of valid places in each row
    mask = np.arange(lens.max()) < lens[:,None]

    # Setup output array and put elements from data into masked positions
    out = np.full((mask.shape[0], mask.shape[1], 2), [None, None])
    out[mask] = np.concatenate(data)
    return out
        

def drawlines(img1,lines):
    r,c,_ = img1.shape
    for r in lines:
        color = tuple(np.random.randint(0,255,3).tolist())
        x0,y0 = map(int, [0, -r[2]/r[1] ])
        x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
        img1 = cv.line(img1, (x0,y0), (x1,y1), color,1)
    return img1


def make_square(img):
    x, y, _ = img.shape
    size = max(x, y)
    new_img = np.zeros((size, size, 3), dtype=np.uint8)
    ax,ay = (size - img.shape[1])//2,(size - img.shape[0])//2
    new_img[ay:img.shape[0]+ay,ax:ax+img.shape[1]] = img
    return new_img


def camera_pose_to_serializable(camera_poses):
    for i in range(0, len(camera_poses)):
        camera_poses[i] = {k: v.tolist() for (k, v) in camera_poses[i].items()}

    return camera_poses


if __name__ == '__main__':
    socketio.run(app, port=3001, debug=True)