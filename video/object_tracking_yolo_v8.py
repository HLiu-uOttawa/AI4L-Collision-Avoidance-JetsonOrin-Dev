from tracking.DetectionsAtTime import DetectionDetails, DetectionsAtTime
from ultralytics import YOLO
import time
from datetime import datetime
from PIL import Image
import os
import multiprocessing as mp
import math
from constants import IMAGE_DETECTION_TYPE
import pandas as pd
import re
from video.object_location_size import CameraDetails, object_location
from video.VideoConfiguration import VideoConfiguration
import threading

import cv2

from pyzbar.pyzbar import decode
import cv2


def read_qrcode_from_image(np_image):
    # Ensure the image is either grayscale or color.
    decoded_objects = decode(np_image)
    for obj in decoded_objects:
        data = obj.data.decode("utf-8")
        return data  # Return the content of the first QR code.
    return None


def extract_timestamp_from_filename(filename):
    """
    Extract the last timestamp from a file or folder path.
    If multiple timestamps are found, the last one is returned.
    """
    # Regex pattern for timestamps with optional milliseconds
    timestamp_pattern = r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}(?:\.\d{3})?)'

    # Find all matches for the timestamp pattern
    matches = re.findall(timestamp_pattern, filename)
    if matches:
        # Get the last match (or the second one explicitly, if needed)
        timestamp_str = matches[-1]  # Change to matches[1] if you want the second explicitly

        # Replace underscores with spaces to match datetime format
        timestamp_str = timestamp_str.replace('_', ' ')

        # Parse the timestamp (check if milliseconds are included)
        if '.' in timestamp_str:
            dt = datetime.strptime(timestamp_str, '%Y-%m-%d %H-%M-%S.%f')
        else:
            dt = datetime.strptime(timestamp_str, '%Y-%m-%d %H-%M-%S')

        # Convert to Pandas Timestamp
        return pd.Timestamp(dt)
    else:
        raise ValueError(f"No valid timestamp found in path: {filename}")


def setup_output_folders(output_directory: str, save_raw_img: bool = True, start_time: pd.Timestamp = None):
    """
    Create the output folder structure for the run
    Args:
        output_directory (str): the root folder to save the results
        save_raw_img (bool): if True, save the original images to disk
        start_time (datetime): the time the run started, used to create a unique folder name
    Returns:
        output_folder (str): the path to the folder where the run results will be saved
    """

    if start_time is None:
        start_time_str = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    else:
        start_time_str = start_time.strftime('%Y-%m-%d_%H-%M-%S')

    output_folder = os.path.join(output_directory, start_time_str)
    os.makedirs(output_folder, exist_ok=True)

    if (save_raw_img):
        # Create a folder to save the raw output images if the option is selected
        raw_output_folder = os.path.join(output_folder, "raw")
        os.mkdir(raw_output_folder)
        print("Saving video tracking contents of the run to: ", raw_output_folder)

    return output_folder


def detection_from_bbox(yolo_box, detected_object, camera_details: CameraDetails,
                        print_details=False) -> DetectionDetails:
    """
    Calculate the object location and size from the bounding box coordinates.
    
    :param yolo_box: Yolo bounding box definition
    :param detected_object: The object detected
    :param camera_details: A CameraDetails object containing details of the camera.
    :return: Detection object containing the object type and location.
    """
    # print(f"Object: {detected_object}, Confidence { yolo_box.conf[0].item()}")

    x1, y1, x2, y2 = yolo_box.xyxy[0].tolist()
    polar_range_data = object_location(x1, y1, x2, y2, camera_details=camera_details, detected_object=detected_object)

    # Convert angles from degrees to radians
    az_angle_rad = math.radians(polar_range_data[0])
    el_angle_rad = math.radians(polar_range_data[1])
    distance = polar_range_data[2]
    if print_details:
        print(
            f"Object: {detected_object}, Distance: {distance}, Azimuth Deg: {polar_range_data[0]}, Elevation Deg: {polar_range_data[1]}")

    # Calculate the horizontal distance (on the ground)
    distance_horizontal = distance * math.cos(el_angle_rad)

    # Calculate x (horizontal) and y (vertical) distances
    x = distance_horizontal * math.sin(az_angle_rad)  # Horizontal distance in the x direction
    y = distance * math.sin(el_angle_rad)  # Vertical distance in the y direction

    return DetectionDetails(detected_object,
                            polar_range_data)  # DetectionDetails(detected_object, [x, 0.2, y, 0.2]) #TODO Return Detect details on camera



# ------------------------------ Async image saving ------------------------------
def save_frame_async(image, filename):
    """
    Asynchronously save a PIL Image to disk without blocking the main thread.

    Args:
        image (PIL.Image.Image): The image to save.
        filename (str): The target file path for saving the image.
    """
    def worker():
        image.save(filename)
    threading.Thread(target=worker, daemon=True).start()

# ------------------------------ track_objects ------------------------------
from multiprocessing.managers import ListProxy

def track_objects(stop_event,
                  video_config: VideoConfiguration,
                  start_time: pd.Timestamp,
                  data_queue: mp.Queue = None,
                  shared_buffers: dict[str, dict[str, ListProxy | int]] = None):
    """
    Track objects using YOLO detection algorithm.
    """
    ### PARAMs to the program
    model_weights = video_config.modelWeights
    save_raw_img = video_config.saveRawImages
    save_detection_video = video_config.saveProcessedVideo
    output_directory = video_config.outputDirectory
    source = video_config.videoSource

    confidence_threshold = video_config.confidenceThreshold

    iou_threshold = video_config.iouThreshold
    stream = video_config.videoStream
    # Graphical show users
    show_boxes = video_config.showBoxesInVideo
    show = video_config.showVideo
    ### END OF PARAMS

    if source == "" or source is None:
        raise Exception("No video source provided. Please provide a source for the video.")

    camera = video_config.camera_details

    output_folder = setup_output_folders(output_directory, save_raw_img, start_time)

    model = YOLO(model_weights).to('cuda')
    print(f"Opening video Source: {source}")

    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"Failed to open video source: {source}")
        return

    is_camera = str(source).isdigit() or str(source).startswith("/dev/video")
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            if is_camera:
                print("Camera frame not available, retrying...")
                time.sleep(0.1)
                continue
            else:
                print("Video playback ended.")
                break

        results = model.track(frame, persist=True, conf=confidence_threshold, iou=iou_threshold, verbose=False)

        buf = shared_buffers["image"]["data"]
        buf.append((time.time(), {"frame": frame}))
        if len(buf) > shared_buffers["image"]["maxlength"]:
            buf.pop(0)


        for i, result in enumerate(results):
            # If configured, saved the original image to disk, in the <output_folder>/raw/*
            if save_raw_img:
                orig_img_rgb = Image.fromarray(result.orig_img[..., ::-1])  # Convert BGR to RGB

                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S.%f")[:-3]
                filename = os.path.join(output_folder, "raw", f"frame_{i:05d}_{timestamp}.png")
                save_frame_async(orig_img_rgb, filename)

            detection_timestamp = datetime.now().replace(microsecond=0)
            detections = []

            # Iterate over the detected objects, add tracking details into the detections_data list
            for box in result.boxes:
                classification_index = box.cls[0].item()
                detected_object = result.names[classification_index]
                # print(f"Object: {detected_object}, Confidence { box.conf[0].item()}")

                detection = detection_from_bbox(box, detected_object, camera_details=camera,
                                                print_details=video_config.printDetectedObjects)

                detections.append(detection)

            # If a data_queue is provided, put the detections into the queue
            if data_queue is not None and len(detections) > 0:
                data_queue.put(DetectionsAtTime(detection_timestamp, IMAGE_DETECTION_TYPE, detections))

            # time.sleep(video_config.videoDelayBetweenProcessingSec)

    # ------------------------------------------------------------------------------------------------
    # model.add_callback("on_predict_batch_end", on_predict_batch_end)
    # results = model.track(source=source,
    #                       conf=confidence_threshold,
    #                       iou=iou_threshold,
    #                       save=save_detection_video,
    #                       show=show,
    #                       stream=stream,
    #                       project=output_folder,
    #                       show_boxes=show_boxes)


if __name__ == "__main__":
    video_config = VideoConfiguration()
    track_objects(video_config, None)
