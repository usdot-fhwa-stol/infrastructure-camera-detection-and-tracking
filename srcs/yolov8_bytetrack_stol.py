from models import TRTModule
import argparse
from time import time
import cv2
from pathlib import Path
import torch
import ctypes
from bytetrack.byte_tracker import BYTETracker

from config import CLASSES, COLORS, H_MATRIX, DET_ZONE
from models.torch_utils import det_postprocess
from models.utils import blob, letterbox, path_to_list
from datetime import datetime, timedelta
import json
import numpy as np
import random
from math import radians, cos, sin, asin, sqrt
import math
from pyproj import Transformer
import csv
from geopy.distance import geodesic
import pandas as pd
#import socket ##uncomment this if you want to tranmit udp to other device

import threading
from flask import Flask, Response, render_template, jsonify

app = Flask(__name__)

# Global variable to store the latest frame
latest_frame = None
frame_lock = threading.Lock()
tracked_objects = []
conflict_list = []

class ROI:
    def __init__(self, x1, y1, x2, y2, roi_id):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.roi_id = roi_id
        self.count = 0


DICT_ROIS = {}
DEBOUNCE_PERIOD = timedelta(seconds=2)
person_tracker = {}
debounce_tracker = {}

color_dict = {}


# All 0.9052174687385559
# Insanely good with 0.25 height offset
K = np.array([[8.94429165e+02, 0.00000000e+00, 6.45495370e+02],
 [0.00000000e+00, 1.12363936e+03, 4.20210159e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
d = np.array([-0.51498051,  0.10524621, -0.00603029, -0.02139855,  0.00616998])
newcameramatrix = np.array([[387.33544107,   0.,         534.09059684],
 [  0.,         505.10401128, 436.17367482],
 [  0.,           0.,           1.        ]])
H = np.array([[ 3.31445642e-02,  4.00938084e-01, -1.22143253e+02],
 [-1.29239710e-02, -1.56338683e-01,  4.76273545e+01],
 [-2.71362493e-04, -3.28251998e-03,  1.00000000e+00]])

def vehicle_center_to_latlon(center, bbox):
    image_cx, image_cy = center
    x1, y1, x2, y2 = bbox
    bbox_height = np.abs(y2 - y1)
    distorted_points = np.float32(np.vstack((image_cx, image_cy + bbox_height*0.25)).T).reshape(-1, 1, 2)
    distorted_points = np.float32(np.vstack((image_cx, image_cy)).T).reshape(-1, 1, 2)
    image_coords_und = cv2.undistortPoints(distorted_points, K, d, P=newcameramatrix)
    latlon_coords = cv2.perspectiveTransform(image_coords_und, H)
    lon_offset = 6.677e-06
    lat_offset = 4.500e-06
    lon_final = latlon_coords[0, 0, 0] + lon_offset # 0.5, 0.75 0.905
    lat_final = latlon_coords[0, 0, 1] - lat_offset # 0.5, 0.75 0.905
    # print(f'final lat/lon: [{lat_final}, {lon_final}]')
    # must_sensor_dist = haversine_distance(MUST_sensor_loc[1], MUST_sensor_loc[0], lat_final, lon_final)
    # print(f'distance from MUST sensor: {must_sensor_dist}')

    return (lat_final, lon_final)


def get_random_color(id):
    if id not in color_dict:
        color_dict[id] = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
    return color_dict[id]


def calculate_center_point(bbox):
    x1, y1, x2, y2 = bbox
    return ((x1 + x2) / 2, (y1 + y2) / 2)


def xy2latlon(point):
    #homography_matrix
    homography_matrix_obtained = H_MATRIX
    mat = homography_matrix_obtained
    matinv = np.linalg.inv(mat)#.I
    # Convert to lat/lon
    point_3D = [point[0], point[1], 1]
    hh = np.dot(matinv,point_3D)
    scalar = hh[2]
    latitude = hh[0]/scalar
    longitude = hh[1]/scalar
    return (latitude, longitude)


def latlon2xy(latlon_point):
    # Homography matrix
    homography_matrix = H_MATRIX

    # Convert latitude/longitude to pixel coordinates
    point_3D = [latlon_point[0], latlon_point[1], 1]  # Homogeneous coordinates (lat, lon, 1)
    pixel_coordinates = np.dot(homography_matrix, point_3D)  # Apply homography

    # Normalize by the scalar value
    scalar = pixel_coordinates[2]
    x = int(pixel_coordinates[0] / scalar)
    y = int(pixel_coordinates[1] / scalar)

    return (abs(x), abs(y))


# Function to calculate the great circle distance between two points on Earth
def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points
    on the Earth (specified in decimal degrees).
    """
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * asin(sqrt(a))
    r = 6371  # Radius of Earth in kilometers
    return c * r * 1000  # Convert to meters


def calculate_bearing(prev, curr):
    """
    Calculate the bearing angle from point (lat1, lon1) to point (lat2, lon2)
    Latitude and longitude should be in radians.
    """
    # Convert latitude and longitude from degrees to radians
    lat1 = prev[0]; lon1 = prev[1]; lat2 = curr[0]; lon2 = curr[1]
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # Calculate the difference in longitude
    delta_lon = lon2 - lon1

    # Calculate bearing angle
    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)

    # Use atan2 to compute the bearing
    bearing_radians = math.atan2(x, y)

    # Convert the bearing to degrees
    bearing_degrees = math.degrees(bearing_radians)

    # Normalize the bearing to a 0-360 degree range
    bearing_degrees = (bearing_degrees + 360) % 360

    return bearing_degrees


def calculate_speed(prev_center, current_center, time_diff):
    dx = current_center[0] - prev_center[0]
    dy = current_center[1] - prev_center[1]
    # distance = math.sqrt(dx**2 + dy**2)
    distance = haversine(current_center[1], current_center[0], prev_center[1], prev_center[0])
    # return distance / (1/13) if time_diff > 0 else 0
    return distance / time_diff if time_diff > 0 else 0


def process_timestamp(time_str):
    time_format = "%Y-%m-%d %H:%M:%S.%f"
    time_obj = datetime.strptime(time_str, time_format)
    timestamp_in_seconds = time_obj.timestamp()
    timestamp_in_seconds_with_milliseconds = round(timestamp_in_seconds, 3)
    return timestamp_in_seconds_with_milliseconds


def predict_position(lat, lon, speed, heading, time_step):
    """
    Predicts future latitude and longitude based on speed and heading.
    - lat, lon: current latitude and longitude (degrees)
    - speed: current speed in meters/second
    - heading: current heading in degrees
    - time_step: time step in seconds to predict future position
    Returns future (latitude, longitude).
    """
    # Convert heading to radians
    heading_rad = np.radians(heading)

    # Calculate distance traveled in time_step (in meters)
    distance_traveled = speed * time_step

    # Calculate delta latitude and delta longitude based on the heading
    delta_lat = (distance_traveled * np.cos(heading_rad)) / 111320  # Approx 111,320 meters per degree of latitude
    delta_lon = (distance_traveled * np.sin(heading_rad)) / (111320 * np.cos(np.radians(lat)))

    # New predicted position (latitude, longitude)
    new_lat = lat + delta_lat
    new_lon = lon + delta_lon

    return new_lat, new_lon


def compute_ttc(lat1, lon1, lat2, lon2, speed1, speed2, heading1, heading2, radius, time_horizon=2, time_step=0.1):
    """
    Computes TTC by simulating future trajectories of two vehicles.
    - radius: Collision threshold based on the sum of the vehicle's radius (in meters)
    - time_horizon: Maximum time horizon to simulate (seconds)
    - time_step: Time step for each simulation step (seconds)
    Returns the TTC if a collision is predicted, or infinity if no collision.
    """
    for t in np.arange(0, time_horizon, time_step):
        # Predict future positions of both vehicles
        future_lat1, future_lon1 = predict_position(lat1, lon1, speed1, heading1, t)
        future_lat2, future_lon2 = predict_position(lat2, lon2, speed2, heading2, t)

        # Calculate the distance between the two future positions
        distance = geodesic((future_lat1, future_lon1), (future_lat2, future_lon2)).meters

        # Check if distance is less than the collision radius (sum of the two vehicle's radius)
        if distance < 2 * radius:
            conflict_point = ((future_lat1 + future_lat2) / 2, (future_lon1 + future_lon2) / 2)
            return t, conflict_point  # Return the time to collision and conflict point

    return float('inf'), None  # No collision predicted within the time horizon


def detect_conflict_with_prediction(vehicles, radius=1, ttc_threshold=0.5):
    """
    Detects potential conflicts between vehicles using linear trajectory prediction.
    - vehicles: List of vehicle data [(lat, lon, speed, heading), ...]
    - radius: Average radius of a vehicle in meters (assume circular vehicle shape)
    - ttc_threshold: TTC threshold for predicting a conflict (seconds)
    Returns a list of conflicts (vehicle index pairs and TTC).
    """
    conflicts = []
    key_list = list(vehicles.keys())
    for i in range(len(key_list)):
        for j in range(i + 1, len(key_list)):
            vehicle1 = vehicles[key_list[i]]
            vehicle2 = vehicles[key_list[j]]

            # Extract latitude, longitude, speed, and heading
            lat1, lon1, speed1, heading1 = vehicle1
            lat2, lon2, speed2, heading2 = vehicle2

            # Compute TTC between the two vehicles
            ttc, conflict_point = compute_ttc(lat1, lon1, lat2, lon2, speed1, speed2, heading1, heading2, radius)

            # Check if TTC is below the threshold
            if ttc < ttc_threshold and ttc > 0:
                conflicts.append((key_list[i], key_list[j], conflict_point, ttc))

    return conflicts


def main(args):
    global latest_frame
    args_bytetrack = argparse.Namespace()
    args_bytetrack.track_thresh = 0.2
    args_bytetrack.track_buffer = 200
    args_bytetrack.mot20 = True
    args_bytetrack.match_thresh = 0.7

    tracker = BYTETracker(args_bytetrack)
    device = torch.device(args.device)
    Engine = TRTModule(args.engine, device)
    H, W = Engine.inp_info[0].shape[-2:]

    Engine.set_desired(['num_dets', 'bboxes', 'scores', 'labels'])

    ####==================================
    ##Uncomment this following block if you want to transmit udp
    ## Set up UDP socket
    #UDP_IP = "128.95.204.54" ## define your receiver IP address
    #UDP_PORT = 5005
    #sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ####==================================

    fps = 0
    # input video
    cap = cv2.VideoCapture(args.vid)
    video_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    video_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    filename = args.output_filename
    path_save_results = args.result_dir + 'results'
    
    if args.store_video:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(path_save_results + "/" + filename + "_detections.mp4", fourcc, 13, (video_width, video_height))

    if args.udp_save:
        filename = args.output_filename
        path_save_results = args.result_dir + 'results'
        file_speed_w = open(path_save_results + "/" + filename + "_results.txt", "w")

    if args.recorded_time:
        if args.timestamp_file is None:
            print(f'Please enter the name of the timestamps file with "--timestamp_file $(name.csv)"')
            exit()
        df = pd.read_csv(args.timestamp_file)


    roi_vertices = np.array(DET_ZONE, np.int32)

    prev_centers = {}
    prev_times = {}

    counter = 0

    while(True):
        ret, frame = cap.read()

        if frame is None:
            print('No image input!')
            break


        if args.recorded_time:
            row = df[df['Frame'] == counter]
            current_time_ori = row['Timestamp'].values[0]
            current_time = process_timestamp(current_time_ori)
            counter += 1
        else:
            current_time = time()


        start = float(time())
        fps_str = "FPS:"
        fps_str += "{:.2f}".format(fps)
        bgr = frame
        bgr, ratio, dwdh = letterbox(bgr, (W, H))
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        tensor = blob(rgb, return_seg=False)

        dwdh = torch.asarray(dwdh * 2, dtype=torch.float32, device=device)

        tensor = torch.asarray(tensor, device=device)

        data = Engine(tensor)
        bboxes, scores, labels = det_postprocess(data)
        # print(labels)

        if bboxes.numel() == 0:
            continue

        bboxes -= dwdh
        bboxes /= ratio

        output = []
        vehicles = {}

        for (bbox, score, label) in zip(bboxes, scores, labels):
            if (label in [0,1,2,3,5,7] and score.item() > 0.2):
                bbox = bbox.round().int().tolist()
                cls_id = int(label)

                cls = CLASSES[cls_id]
                # x1, y1, x2, y2, conf
                output.append([bbox[0], bbox[1], bbox[2], bbox[3], score.item(), cls_id])

        output = np.array(output)

        info_imgs = frame.shape[:2]
        img_size = info_imgs

        if output != []:
            online_targets = tracker.update(output, info_imgs, img_size)
            tracked_objects.clear()
            conflict_list.clear()
            online_tlwhs = []
            online_ids = []
            online_scores = []
            online_cls = []
            for t in online_targets:
                tlwh = t.tlwh
                tid = t.track_id
                bbox = [tlwh[0], tlwh[1], tlwh[0] + tlwh[2], tlwh[1] + tlwh[3]]

                class_index = t.cls
                class_name = CLASSES[int(class_index)]
                online_tlwhs.append(tlwh)
                online_ids.append(tid)
                online_scores.append(t.score)
                online_cls.append(class_name)

                center_2D = calculate_center_point(bbox)
                # center = xy2latlon(center_2D)
                center = vehicle_center_to_latlon(center_2D, bbox)

                if tid in prev_centers and tid in prev_times:
                    heading = calculate_bearing(prev_centers[tid], center)
                    speed = calculate_speed(prev_centers[tid], center, current_time - prev_times[tid])
                else:
                    heading = 0
                    speed = 0

                size = ""
                if int(class_index) in [0, 1, 3]:
                    size = 'small'
                elif int(class_index) in [2]:
                    size = 'medium'
                elif int(class_index) in [5, 7]:
                    size = 'large'
                else:
                    size = 'N/A'

                list_tem = str(current_time).split(".")
                list_tem[1] = list_tem[1][0:3]
                timestamp = str(list_tem[0] + '.' + list_tem[1])

                lat_origin, lon_origin = 47.6277146, -122.1432525
                lat_target, lon_target = center[0], center[1]
                transformer = Transformer.from_crs("epsg:4326", "epsg:32610")
                x_origin, y_origin = transformer.transform(lat_origin, lon_origin)
                x_target, y_target = transformer.transform(lat_target, lon_target)

                dx = str(x_target - x_origin)  # "X difference (East): {dx} meters"
                dy = str(y_target - y_origin)  # "Y difference (North): {dy} meters"

                prev_centers[tid] = center
                prev_times[tid] = current_time

                if cv2.pointPolygonTest(roi_vertices, center_2D, False) >= 0:

                    vehicles[tid] = (center[0], center[1], speed, heading)

                    if args.udp_save:
                        # Needs to update
                        file_speed_w.write(f"{class_name},{dx},{dy},{heading},{speed},{size},{float(t.score) * 100},{int(tid)},{timestamp}\n")

                    # if args.show:
                    cv2.rectangle(frame, (int(tlwh[0]), int(tlwh[1])), (int(tlwh[0] + tlwh[2]), int(tlwh[1] + tlwh[3])), get_random_color(tid), 2)
                    cv2.putText(frame, str(tid), (int(tlwh[0]), int(tlwh[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    tracked_objects.append({
                    'track_id': tid,
                    'class_name': class_name,
                    'speed': speed,
                    'heading': heading,
                    'size': size,
                    'score': float(t.score)*100,
                    })
            if args.store_video:
                out.write(frame)
            if args.display_video:
                cv2.imshow('image', frame)
                cv2.waitKey(1)

            # # Detect potential conflicts
            # conflicts = detect_conflict_with_prediction(vehicles)
            # # conflict_list.append(conflicts)
            # for vehicle_1, vehicle_2, conflict_loc, ttc in conflicts:
            #     conflict_list.append({
            #         'vehicle_1': vehicle_1,
            #         'vehicle_2': vehicle_2,
            #         'conflict_loc': conflict_loc,
            #         'ttc': ttc,
            #     })
            #
            # if conflicts:
            #     print(f"Potential conflicts detected: {conflicts}")



                # print('conflict_list : ', conflict_list)
                # print('=============================')
            # else:
            #     print("No conflicts detected.")

            #####=======================================
            #### Uncomment the following code to visualize Output conflicts
            # if conflicts:
            #     print(f"Potential conflicts detected: {conflicts}")
            #     # (key_list[i], key_list[j], conflict_point, ttc)
            #     for conflict in conflicts:
            #         print(latlon2xy(conflict[2]))
            #         vehicle_1 = latlon2xy(prev_centers[conflict[0]])
            #         vehicle_2 = latlon2xy(prev_centers[conflict[1]])
            #         conflict_point = latlon2xy(conflict[2])
            #         vehicle_1_pred = predict_position(prev_centers[conflict[0]][0], prev_centers[conflict[0]][1], vehicles[conflict[0]][2], vehicles[conflict[0]][3], 0.5)
            #         vehicle_2_pred = predict_position(prev_centers[conflict[1]][0], prev_centers[conflict[1]][1], vehicles[conflict[1]][2], vehicles[conflict[1]][3], 0.5)
            #
            #
            #         cv2.circle(frame, conflict_point, radius=5, color=(0, 0, 255), thickness=1)
            #         cv2.circle(frame, vehicle_1, radius=5, color=(0, 255, 0), thickness=1)
            #         cv2.circle(frame, vehicle_2, radius=5, color=(0, 255, 0), thickness=1)
            #         cv2.line(frame, vehicle_1, latlon2xy(vehicle_1_pred), color=(0, 255, 0), thickness=1)
            #         cv2.line(frame, vehicle_2, latlon2xy(vehicle_2_pred), color=(0, 255, 0), thickness=1)
            #
            #     output_filename = f"./conflict/conflict_{current_time_ori}.jpg"
            #     cv2.imwrite(output_filename, frame)
            # else:
            #     print("No conflicts detected.")
            #####=======================================


        ###=======================================
        ## Uncomment this if you want to transmit udp data to other device
        ## Serialize and send data over UDP
        # json_data = json.dumps(tracked_objects)
        # sock.sendto(json_data.encode(), (UDP_IP, UDP_PORT))
        ###=======================================

        # out.write(frame)
        with frame_lock:
            latest_frame = frame.copy()

    cap.release()
    if args.store_video:
        out.release()
    cv2.destroyAllWindows()

def generate_frames():
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is not None:
                frame = latest_frame.copy()
            else:
                continue

        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
@app.route('/tracked_objects')
def get_tracked_objects():
    global tracked_objects
    return jsonify(tracked_objects)

## For speed up you may not want to transmit conflict data (most of the time no conflict actually), you can comment out this
@app.route('/conflict_data')
def get_conflict_data():
    global conflict_list
    return jsonify(conflict_list)
##===

@app.route('/')
def index():
    return render_template('index.html')



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--engine', type=str, help='Engine file', default='../models/engine/yolov8n.engine')
    parser.add_argument('--vid', type=str, help='Video files', default='../sample_video/2024_08_28_13_03_00_raw.mp4') # for video stream define your rtsp protocol
    parser.add_argument('--device', type=str, default='cuda:0', help='TensorRT infer device')
    parser.add_argument('--udp_save', action='store_true', help='Save the 9 outputs in txt files')
    parser.add_argument('--result_dir', type=str, help='UDP save directory', default='../sample_video/test')
    parser.add_argument('--output_filename', type=str, help='Filename for output', default='')
    parser.add_argument('--recorded_time', action='store_true', help='Input timestamps from a file, when processing raw data')
    parser.add_argument('--timestamp_file', type=str, help='Filename for timestamps csv, when processing raw data')
    parser.add_argument('--display_video', action='store_true', help='View the video live for debugging')
    parser.add_argument('--store_video', action='store_true', help='Store video with detected objects')

    args = parser.parse_args()
    main(args)

# ## Start the web app
    threading.Thread(target=main, args=(args,), daemon=True).start()
    app.run(host='0.0.0.0', port=5000)
