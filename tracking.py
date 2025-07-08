import multiprocessing as mp
import numpy as np
import time
import torch
import pandas as pd
import os
# import cv2
import csv
from datetime import datetime, timedelta
from collections import deque
from datetime import datetime, timedelta
from multiprocessing.managers import ListProxy

from video.VideoConfiguration import VideoConfiguration
from tracking.TrackingConfiguration import TrackingConfiguration

from cli_arguments import define_argument_parser, update_radar_config, update_video_config, update_tracking_config

# Different tracking programs
from tracking.ObjectTrackingGmPhd import get_object_tracking_gm_phd
from radar.configuration.RadarConfiguration import RadarConfiguration
from radar.radar_tracking import RadarTracking
from video.object_tracking_yolo_v8 import track_objects

from imu import imu
from gps import gps

from datetime import datetime, timedelta
import time

import multiprocessing as mp
import queue
from datetime import datetime, timedelta

import multiprocessing as mp
import time
from typing import Any, Dict, List, Tuple
import socket, json  # added for communication to server by socket

from utils.log_utils import log_print


class MultiSensorBuffer:
    """
    A shared memory buffer to store multiple sensor types (images, radar frames, IMU, GPS),
    each with its own independent rolling time window.

    This class can only work on Linux-like os
    """

    def __init__(self, time_window_sec: float = 1.3):
        self.time_window = time_window_sec
        self._manager = mp.Manager()

        self.buffers = {
            "image": self._manager.list(),
            "radar": self._manager.list(),
            "imu": self._manager.list(),
            "gps": self._manager.list(),
        }  # type: Dict[str, Any]

    def append(self, sensor_type: str, data: Any) -> None:
        """
        Append data to the buffer of a specific sensor type.

        Args:
            sensor_type (str): One of 'image', 'radar', 'imu', 'gps'.
            data (Any): The sensor data (with implicit current timestamp).
        """
        if sensor_type not in self.buffers:
            raise ValueError(f"Invalid sensor type: {sensor_type}")

        timestamp = time.time()
        self.buffers[sensor_type].append((timestamp, data))
        self._trim_old(sensor_type, timestamp)

    def get_latest(self, sensor_type: str) -> List[Tuple[float, Any]]:
        """
        Retrieve the current valid data in buffer for a sensor type.

        Args:
            sensor_type (str): One of 'image', 'radar', 'imu', 'gps'.

        Returns:
            List[Tuple[float, Any]]: List of (timestamp, data) within the time window.
        """
        if sensor_type not in self.buffers:
            raise ValueError(f"Invalid sensor type: {sensor_type}")
        return list(self.buffers[sensor_type])

    def _trim_old(self, sensor_type: str, current_time: float) -> None:
        """
        Trim outdated data from buffer based on the time window.

        Args:
            sensor_type (str): Sensor type buffer to trim.
            current_time (float): Current timestamp.
        """
        buffer = self.buffers[sensor_type]
        cutoff = current_time - self.time_window
        while len(buffer) > 0 and buffer[0][0] < cutoff:
            buffer.pop(0)


# Image buffer for 1.3 seconds (@10 FPS)
# Radar buffer for 1.3 seconds (@25 Hz)
# IMU buffer for 1.3 seconds (@200 Hz)
# GPS buffer for 1.3 seconds (@5 Hz)


def rawdata_buffer_task(stop_event, buffers):
    while not stop_event.is_set():
        now = time.time()
        # buffers["image"].append((now, {"frame": f"img@{now}"}))
        # buffers["radar"].append((now, {"radar": f"r@{now}"}))
        # buffers["imu"].append((now, {"imu": [0.1, 0.2, 9.8]}))
        # buffers["gps"].append((now, {"lat": 45.42, "lon": -75.69}))
        # print(f"Image info in buffers... length: {len(buffers['image']['data'])}")
        # print(f"Radar info in buffers... length: {len(buffers['radar']['data'])}")
        # print(buffers['image'])
        # print(buffers['image'])

        # if len(buffers[key]) > MAX_LEN[key]:
        #     buffers[key].pop(0)

        time.sleep(0.1)


#  -------------------------------------------------------------------------------  #
detection_history = deque()
avoidance_flag_sent = {2: False, 1: False}
detection_window_seconds = 1.0  # Time window in seconds, 1 s by default

def check_and_send_avoidance_flag(current_time: datetime) -> int:
    """
    Checks for detections at the current time point and triggers appropriate avoidance levels
    based on historical records and the defined time window.

    Core Logic:
    1. Maintain a history queue of active detections within the specified time window.
    2. If 2 detections occur within the current 1-second window, trigger Level 2 avoidance
       and immediately clear the history queue.
    3. Subsequently, if 2 detections accumulate again within a new 1-second window
       (after Level 2 triggered and cleared the queue), trigger Level 1 avoidance.

    Args:
        current_time: The timestamp of the current detection (must be a datetime object).

    Returns:
        int: Returns 2 if Level 2 is triggered, 1 if Level 1 is triggered;
             Returns 0 if no avoidance level is triggered.
    """
    global detection_history, avoidance_flag_sent

    # 1. Add the current detection timestamp to the end of the queue.
    detection_history.append(current_time)

    # 2. Remove outdated timestamps that fall outside the current time window.
    # This loop ensures that the detection_history queue only contains detection records
    # that are within the detection_window_seconds range relative to the current_time.
    while detection_history and \
          (current_time - detection_history[0]).total_seconds() > detection_window_seconds:
        detection_history.popleft() # Remove the oldest element from the front of the queue

    # 3. Calculate the number of valid detections within the current time window.
    current_detection_count = len(detection_history)

    # --- Trigger Level 2 Avoidance ---
    # Level 2 will be triggered only if all the following conditions are met:
    #   - There are exactly 2 detections within the current 1-second window (current_detection_count == 2).
    #   - The Level 2 avoidance flag has not been sent yet (not avoidance_flag_sent[2]), to prevent re-triggering.
    if current_detection_count == 2 and not avoidance_flag_sent[2]:
        print(f"[!] Triggering Level 2 Avoidance at {current_time.strftime('%Y-%m-%d %H:%M:%S.%f')}")
        sent_avoidance_flag(level=2) # Call the function to send the avoidance flag
        avoidance_flag_sent[2] = True # Mark Level 2 as sent, to prevent further triggers

        # **Core logic: Immediately clear the history queue after Level 2 is triggered.**
        # This is to "reset" the count and provide a clean context for Level 1 triggering.
        detection_history.clear()
        # print("    -> detection_history cleared after Level 2 trigger.")

        # **Important: Reset the Level 1 sent status.**
        # This allows Level 1 to be triggered in a new counting cycle after Level 2 has been triggered and the queue cleared.
        avoidance_flag_sent[1] = False
        return 2 # Return the triggered level

    # --- Trigger Level 1 Avoidance ---
    # Level 1 will be triggered only if all the following conditions are met:
    #   - Level 2 avoidance has already been triggered (avoidance_flag_sent[2] is True).
    #     This ensures that Level 1 is an "escalation" event that occurs after Level 2.
    #   - After Level 2 was triggered and the queue cleared, 2 detections accumulate again
    #     within the current new 1-second window (current_detection_count == 2).
    #   - The Level 1 avoidance flag has not been sent yet (not avoidance_flag_sent[1]), to prevent re-triggering.
    if avoidance_flag_sent[2] and \
       current_detection_count == 2 and \
       not avoidance_flag_sent[1]:
        print(f"[!!!] Triggering Level 1 Avoidance at {current_time.strftime('%Y-%m-%d %H:%M:%S.%f')}")
        sent_avoidance_flag(level=1) # Call the function to send the avoidance flag
        avoidance_flag_sent[1] = True # Mark Level 1 as sent
        # Depending on requirements, you might also choose to clear the queue after Level 1 is triggered.
        # If you want to completely "reset" the system or prevent further cascading triggers after Level 1, uncomment the line below:
        # detection_history.clear()
        return 1 # Return the triggered level

    # If none of the above conditions are met, no avoidance level is triggered
    return 0

#  -------------------------------------------------------------------------------  #
def sent_avoidance_flag(ip: str = '127.0.0.1', port: int = 65432, level=2) -> None:

    HOST = 'localhost'
    PORT = port
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            message = {"collision": level}
            s.sendall(json.dumps(message).encode())
            data = s.recv(1024)
            response = json.loads(data.decode())
            print("Server Response:", response)
    except (ConnectionRefusedError, json.JSONDecodeError, socket.error) as e:
        print(f"[Error] Communication failed: {e}")

#  -------------------------------------------------------------------------------
def radar_tracking_task(stop_event,
                        config: RadarConfiguration,
                        start_time: pd.Timestamp,
                        radar_data_queue: mp.Queue,
                        shared_buffers: dict[str, dict[str, ListProxy | int]] = None):
    radar_tracking = RadarTracking(config, start_time, radar_data_queue, shared_buffers)
    radar_tracking.object_tracking(stop_event)


def process_queues(stop_event,
                   tracker,
                   image_data_queue,
                   radar_data_queue,
                   imu_status,
                   gps_current_data,
                   batching_time=1):
    count = 1
    batching_time = .3
    last_print_time = datetime.now()
    last_remove_tracks_time = datetime.now()
    time_window = timedelta(seconds=batching_time)  # Define the window
    max_wait_time = 0.14  # Wax wait time to check data in the queue (just less than half the batching time)

    last_batch_time = None
    
    offline_flag = False # Set to True if you want to run the script in offline mode, change to be place better

    # Buffers to store data for the next processing loop
    image_buffer = []
    radar_buffer = []

    detect_output = [
        ['timestamp_image', 'timestamp_radar', 'azimuth', 'elevation', 'distance_camera', 'distance_radar', 'SNR', 'avoidance_flag']]
    detect_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")[:-3]
    detect_output_file_name = "detect_data_" + detect_timestamp + ".csv"

    # Upload data fusion to output folder
    detect_output_file_path = os.path.join("output/", detect_output_file_name)
    radar_timestamps = []
    image_detections_in_window = []
    radar_detections_in_window = []
    while not stop_event.is_set():
        # uncomment below code to test the info from imu and gps threads
        # print(f"In process_queue: {imu_status}")
        # print(f"In process_queue: {gps_current_data.cur}")

        processed_anything = False

        # Define a batch time window for collecting data
        if last_batch_time is None:
            last_batch_time = datetime.now()
            batch_window_end = last_batch_time + time_window  # Set the upper limit for the current batch

        # Fetch all image data within the batch window
        try:
            # Process buffered image data from the previous loop
            for detectionsAtTime in image_buffer:
                image_timestamp = detectionsAtTime.timestamp
                if image_timestamp <= batch_window_end:
                    image_detections_in_window.append(detectionsAtTime)
                else:
                    # Keep the data for the next loop
                    break

            image_buffer = [d for d in image_buffer if d.timestamp > batch_window_end]

            # Process new data from the queue
            while image_data_queue is not None and not image_data_queue.empty():
                detectionsAtTime = image_data_queue.get(timeout=max_wait_time)
                image_timestamp = detectionsAtTime.timestamp

                if image_timestamp <= batch_window_end:
                    image_detections_in_window.append(detectionsAtTime)
                else:
                    # Buffer data for the next loop
                    image_buffer.append(detectionsAtTime)
                    break

        except queue.Empty:
            pass

        # Fetch all radar data within the batch window
        try:
            # Process buffered radar data from the previous loop
            for detectionsAtTime in radar_buffer:
                radar_timestamp = detectionsAtTime.timestamp
                if radar_timestamp <= batch_window_end:
                    radar_detections_in_window.append(detectionsAtTime)
                else:
                    # Keep the data for the next loop
                    break

            radar_buffer = [d for d in radar_buffer if d.timestamp > batch_window_end]

            # Process new data from the queue
            while radar_data_queue is not None and not radar_data_queue.empty():
                detectionsAtTime = radar_data_queue.get(timeout=max_wait_time)
                radar_timestamp = detectionsAtTime.timestamp

                timestamp_str = radar_timestamp.strftime('%Y-%m-%d_%H-%M-%S')[:-3]
                radar_timestamps.append(timestamp_str)

                if radar_timestamp <= batch_window_end:
                    radar_detections_in_window.append(detectionsAtTime)
                else:
                    # Buffer data for the next loop
                    radar_buffer.append(detectionsAtTime)
                    break

        except queue.Empty:
            pass

        current_time = datetime.now()

        if batch_window_end < current_time:
            last_batch_time = current_time  # Set the start of the next batch window
            batch_window_end = last_batch_time + time_window  # Set the upper limit for the current batch windo

            # Process both image and radar data if available within the time window
            if image_detections_in_window and radar_detections_in_window:
                # print("Data Fusion detected_________________________________________")
                if offline_flag:
                    #If offline mode, we will use the first image detection in the window
                    img_time = image_detections_in_window[0].timestamp
                    img_data = image_detections_in_window[0]
                    
                    
                else:
                    # Get the lastest image detection in the window
                    img_time = image_detections_in_window[-1].timestamp
                    img_data = image_detections_in_window[-1]

                print("Nearest date from camera list :" + str(img_time))
                
                timestamps = []
                for i in range(len(radar_detections_in_window)):
                    timestamps.append(radar_detections_in_window[i].timestamp)

                # Obtain closest timestamps between radar and camera
                cloz_dict = {
                    abs(img_time - date): date
                    for date in timestamps}
                
                res = cloz_dict[min(cloz_dict.keys())]
                print("Nearest date from radar list : " + str(res))
                combination_index = timestamps.index(res)
                
                time_diff = img_time - radar_detections_in_window[combination_index].timestamp 
                print("Time difference:" + str(time_diff.total_seconds()))
                if abs(time_diff.total_seconds()) > 1:
                    print("Time difference is too large, skipping data fusion")

                    if offline_flag:
                        # If offline mode, check if the last image detection is after the last radar detection
                        #If so, pop out first image detection as it does not have matching the radar detection
                        if radar_detections_in_window[-1].timestamp > img_time:
                            if len(image_detections_in_window) > 1:
                                for i in range(1,  len(image_detections_in_window)):  
                                    image_buffer.append(image_detections_in_window[i])
                        else:
                            for i in range(  len(image_detections_in_window)):  
                                image_buffer.append(image_detections_in_window[i])
                        
                else:
                    if offline_flag:
                        print("Offline mode, saving data fusion")
                        if len(image_detections_in_window) > 1:
                            for i in range(1,  len(image_detections_in_window)):  
                                image_buffer.append(image_detections_in_window[i])
                    else:
                        print("data fusion___________________________________________________________")

                    avoidance_flag = check_and_send_avoidance_flag(datetime.now())

                    

                    combined_timestamp = max(
                        img_time, radar_detections_in_window[combination_index].timestamp
                    )
                    save_detections = (
                            [img_time] + [radar_detections_in_window[combination_index].timestamp]  
                            + img_data.detections[0].get_data() + [ radar_detections_in_window[ combination_index].detections[0] ]
                            + [ radar_detections_in_window[ combination_index].detections[1] ] + [avoidance_flag]
                            
                    )
                    
                    detect_output.append(save_detections)
                    combined_detections = [radar_detections_in_window[combination_index], img_data ]
                    
                    combined_timestamp = max(
                        img_time, radar_detections_in_window[combination_index].timestamp
                    )
                    
                    np.savetxt(detect_output_file_path, detect_output, delimiter=",", fmt='% s')

                    #tracker.update_tracks(combined_detections, combined_timestamp, type="combined", print_coord = True)
                    # tracker.update_tracks(combined_detections, combined_timestamp, type="combined") #TODO Implement update tracks
                
                #Offline only: stores radar detections in the radar buffer to ensure synchronization with image data 
                if offline_flag:
                    for i in range(len(radar_detections_in_window)):
                        radar_buffer.append(radar_detections_in_window[i])
                
                # Add the code from Sina to communicate with server
                # import socket
                # import json
                #
                # HOST = 'localhost'
                # PORT = 65432
                #
                # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                #     s.connect((HOST, PORT))
                #     message = {"collision": 1}
                #     s.sendall(json.dumps(message).encode())
                #
                #     data = s.recv(1024)
                #     response = json.loads(data.decode())
                #     print("Server Response:", response)

        
            elif image_detections_in_window:
                print("Image data detected_________________________________________")
                image_timestamp = image_detections_in_window[-1].timestamp
                image_detections = image_detections_in_window[-1].detections

                if offline_flag:
                    for i in range(len(image_detections_in_window)):  
                        image_buffer.append(image_detections_in_window[i])
                # tracker.update_tracks(image_detections, image_timestamp, type="image_only")
                

            elif radar_detections_in_window:
                print("Radar data detected_________________________________________")
                radar_timestamp = radar_detections_in_window[-1].timestamp
                radar_detections = radar_detections_in_window[-1].detections
                radar_buffer_length = len(radar_detections_in_window)

                if offline_flag:
                    for i in range(len(radar_detections_in_window)):
                       radar_buffer.append(radar_detections_in_window[i]) 
       
                # tracker.update_tracks([radar_detections], radar_timestamp, type="radar_only")
                
            else:
                last_batch_time = datetime.now()  # Set the start of the next batch window

                time.sleep(0.01)  # Small sleep to avoid busy waiting
                continue

            #Reset detections for the next window
            image_detections_in_window = []
            radar_detections_in_window = []
            count += 1  
        
        current_time = datetime.now()
        # Print current tracks approx every 5 seconds
        if (current_time - last_print_time).total_seconds() >= 5:
             
            remove = False
            interval = batching_time * 2
            # Print current tracks with remove_tracks=True every 30 seconds, this will remove tracks that are not being updated
            if (current_time - last_remove_tracks_time).total_seconds() >= 30:
                remove = True
                interval = batching_time * 10
                last_remove_tracks_time = current_time

            tracker.print_current_tracks(remove_tracks=remove, interval=interval)
            last_print_time = current_time

    tracker.show_tracks_plot()
    tracker.print_current_tracks(remove_tracks=True, interval=batching_time * 2)


def plot_data(plot_queue: mp.Queue, stop_event):
    while not stop_event.is_set():
        while plot_queue is not None and not plot_queue.empty():
            try:
                plot_data = plot_queue.get(timeout=1)
                # For now, do nothing. 
            except mp.queues.Empty:
                pass


if __name__ == '__main__':

    manager = mp.Manager()

    start_time = pd.Timestamp.now()
    parser = define_argument_parser()
    parser.set_defaults(download=True)
    args = parser.parse_args()

    # print("Starting the tracking processes.")
    log_print("Starting the tracking processes.", log_file_path="logs/common.log")

    # Create a stop event and a queue for data to be passed between processes
    stop_event = mp.Event()
    radar_data_queue = None
    image_data_queue = None
    plot_data_queue = None

    imu_data_queue = None
    gps_data_queue = None

    # imu_status = None
    # gps_current_data = None

    imu_status = manager.Namespace()
    
    gps_current_data = manager.Namespace()
    gps_current_data.cur = None

    # Create a thread to save raw data in shared memory within a time window. 1.3 s
    from utils.buffer_utils import  create_sensor_buffers

    shared_buffers = create_sensor_buffers(manager)
    rawdata_buffer_proc = mp.Process(name="Raw Data Coll.",
                              target=rawdata_buffer_task,
                              args=(stop_event, shared_buffers)
                              )
    rawdata_buffer_proc.start()
    # Generate a IMU Processing
    if not args.skip_imu:
        # print("Process imu and save data...")
        log_print("Process imu and save data...", log_file_path="logs/common.log")
        imu_data_queue = mp.Queue()
        # imu_status = mp.Manager().Namespace()

        imu_proc = mp.Process(
            name="imu_processing",
            target=imu.imu_process,
            args=(stop_event, imu_data_queue, imu_status))
        imu_proc.start()

    # Generate a GPS Processing
    if not args.skip_gps:
        # print("Process gps and save data...")
        log_print("Process gps and save data...", log_file_path="logs/common.log")
        gps_data_queue = mp.Queue()

        gps_proc = mp.Process(
            name="gps_processing",
            target=gps.gps_process,
            args=(stop_event, gps_data_queue, gps_current_data))
        gps_proc.start()

    # Create the video tracking configuration, process, queue to move data
    if not args.skip_video:
        video_config = VideoConfiguration(config_path=args.video_config)
        video_config = update_video_config(video_config,
                                           args)  # Update the video configuration with the command line arguments

        image_data_queue = mp.Queue()
        with torch.no_grad():
            video_proc = mp.Process(name="Video Data Coll.", target=track_objects,
                                    args=(stop_event, video_config, start_time, image_data_queue, shared_buffers))
            video_proc.start()

            # Create the radar tracking configuration, process, queue to move data if not disabled
    if not args.skip_radar:
        if not args.skip_video:  # Add a delay to allow the video process to start before the radar process
            time.sleep(args.radar_start_delay)
        radar_config = RadarConfiguration(config_path=args.radar_config)
        radar_config = update_radar_config(radar_config,
                                           args)  # Update the radar configuration with the command line arguments

        radar_data_queue = mp.Queue()
        radar_proc = mp.Process(name="Radar Data Coll.", target=radar_tracking_task,
                                args=(stop_event, radar_config, start_time, radar_data_queue, shared_buffers))
        radar_proc.start()

    # Create the object tracking configuration, process, queue to move data
    if not args.skip_tracking:
        tracking_config = TrackingConfiguration()
        tracking_config = update_tracking_config(tracking_config,
                                                 args)  # Update the video configuration with the command line arguments
        if not args.skip_radar:
            tracking_config.max_track_distance = radar_config.bin_size_meters * 512  # Override the max distance based on radar range
        tracker = get_object_tracking_gm_phd(start_time, tracking_config)

        # Queue process to handle incoming data
        tracking_proc = mp.Process(name="Tracking",
                                   target=process_queues,
                                   args=(stop_event,
                                          tracker,
                                          image_data_queue,
                                          radar_data_queue,
                                          imu_status,
                                          gps_current_data,
                                          args.batching_time))
        tracking_proc.start()

    try:
        while True:
            user_input = input("Type 'q' and hit ENTER to quit:\n")
            if user_input.lower() == 'q':
                stop_event.set()
                break
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        if not args.skip_radar:
            radar_proc.join()
        if not args.skip_video:
            video_proc.join()
        if not args.skip_tracking:
            tracking_proc.join()

    duration = time.time() - start_time.timestamp()
    # print(f"Tracking duration: {duration:.2f} seconds")
    log_print(f"Tracking duration: {duration:.2f} seconds", log_file_path="logs/common.log")
