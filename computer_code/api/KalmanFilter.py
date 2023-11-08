import cv2 as cv
import numpy as np
from LowPassFilter import LowPassFilter
import time

class KalmanFilter:
    def __init__(self, num_objects):
        state_dim = 9
        measurement_dim = 6
        dt = 0.1
        self.kalmans = []
        self.prev_measurement_time = 0
        self.prev_positions = []

        self.low_pass_filter_xy = []
        self.low_pass_filter_z = []
        self.heading_low_pass_filter = []
        self.num_objects = num_objects

        for i in range(num_objects):
            self.prev_positions.append([0,0,0])
            self.kalmans.append(cv.KalmanFilter(state_dim, measurement_dim))
            self.kalmans[i].transitionMatrix = np.array([[1, 0, 0, dt, 0, 0, 0.5*dt**2, 0, 0],
                                                    [0, 1, 0, 0, dt, 0, 0, 0.5*dt**2, 0],
                                                    [0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt**2],
                                                    [0, 0, 0, 1, 0, 0, dt, 0, 0],
                                                    [0, 0, 0, 0, 1, 0, 0, dt, 0],
                                                    [0, 0, 0, 0, 0, 1, 0, 0, dt],
                                                    [0, 0, 0, 0, 0, 0, 1, 0, 0],
                                                    [0, 0, 0, 0, 0, 0, 0, 1, 0],
                                                    [0, 0, 0, 0, 0, 0, 0, 0, 1]], dtype=np.float32)

            self.kalmans[i].processNoiseCov = np.eye(state_dim, dtype=np.float32) * 1e-2
            self.kalmans[i].measurementNoiseCov = np.eye(measurement_dim, dtype=np.float32) * 1e0
            self.kalmans[i].measurementMatrix = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                                                    [0, 1, 0, 0, 0, 0, 0, 0, 0],
                                                    [0, 0, 1, 0, 0, 0, 0, 0, 0],
                                                    [0, 0, 0, 1, 0, 0, 0, 0, 0],
                                                    [0, 0, 0, 0, 1, 0, 0, 0, 0],
                                                    [0, 0, 0, 0, 0, 1, 0, 0, 0]], dtype=np.float32)
            
            self.kalmans[i].statePost = np.zeros((9,1), dtype=np.float32)


            self.low_pass_filter_xy.append(LowPassFilter(cutoff_frequency=20, sampling_frequency=60.0, dims=2))
            self.low_pass_filter_z.append(LowPassFilter(cutoff_frequency=20, sampling_frequency=60.0, dims=1))
            self.heading_low_pass_filter.append(LowPassFilter(cutoff_frequency=20, sampling_frequency=60.0, dims=1))
    

    def predict_location(self, objects):
        res = []

        dt = time.time() - self.prev_measurement_time
        self.prev_measurement_time = time.time()

        for drone_index in range(0, self.num_objects):
            possible_new_objects = [object for object in objects if object["droneIndex"] == drone_index]
            possible_new_positions = [x["pos"] for x in possible_new_objects]

            if len(possible_new_objects) == 0:
                continue

            kalman = self.kalmans[drone_index]

            kalman.transitionMatrix[:3, 3:6] = dt * np.eye(3)
            kalman.transitionMatrix[3:6, 6:9] = dt * np.eye(3)
            kalman.transitionMatrix[:3, 6:9] = 0.5 * dt**2 * np.eye(3)

            if all(kalman.statePost == 0): # if not initialized
                A = kalman.statePost
                A[0:3] = possible_new_positions[0].reshape((3, 1))
                kalman.statePost = A
                kalman.statePre = A
            
            predicted_location = kalman.predict()[:3].T[0]
            distances_to_predicted_location = np.sqrt(np.sum((possible_new_positions - predicted_location)**2, axis=1))
            closest_match_i = np.argmin(distances_to_predicted_location)
            new_pos = possible_new_positions[closest_match_i].astype(np.float32)
            new_vel = ((new_pos - self.prev_positions[drone_index]) / dt).astype(np.float32)
            self.prev_positions[drone_index] = new_pos

            kalman.correct(np.concatenate((new_pos, new_vel)))
            predicted_state = kalman.statePre[:6].T[0]  # Predicted 3D location

            heading = possible_new_objects[closest_match_i]["heading"]
            heading = self.heading_low_pass_filter[drone_index].filter(heading)[0]
            # heading, self.z = signal.lfilter(self.b, 1, [heading], zi=self.z)

            vel = predicted_state[3:6].copy()
            vel[0:2] = self.low_pass_filter_xy[drone_index].filter(vel[0:2])
            vel[2] = self.low_pass_filter_z[drone_index].filter(vel[2])[0]
            
            res.append({
                "pos": predicted_state[:3],
                "vel": vel,
                "heading": heading,
                "droneIndex": drone_index
            })

        return res


    def reset(self):
        self.prev_measurement_time = time.time()-20

        for i, kalman in enumerate(self.kalmans):
            kalman.statePost = np.zeros((9,1), dtype=np.float32)
            self.prev_positions[i] = np.array([0,0,0])