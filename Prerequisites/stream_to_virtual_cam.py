import os
import cv2
import time
import numpy as np
import pyfakewebcam
from vmbpy import *

# Global variables
fake_cam = None
last_time = 0
frame_interval = 1 / 15  # Limit frame rate to 15 FPS


def setup_camera(cam: Camera):
    with cam:
        try:
            cam.BinningHorizontal.set(3)
            cam.BinningVertical.set(3)
        except:
            print("Binning to 3x3: failed")

        try:
            cmos_width = cam.Width.get()
            cmos_height = cam.Height.get()
            print(f"Camera resolution: {cmos_width} x {cmos_height}")
        except:
            print("Warning: failed to set resolution.")

        try:
            cam.ExposureAuto.set('Continuous')
        except:
            print("Warning: failed to set auto exposure.")

        try:
            cam.BalanceWhiteAuto.set('Once')  # Continuous
        except (AttributeError, VmbFeatureError):
            print("error: cam.BalanceWhiteAuto")
            pass

def frame_handler(cam: Camera, stream: Stream, frame: Frame):
    global fake_cam, last_time

    now = time.time()
    if now - last_time < frame_interval:
        cam.queue_frame(frame)
        return
    last_time = now

    # Get image from frame
    np_image = frame.as_numpy_ndarray()

    # Convert to RGB
    if len(np_image.shape) == 2:
        np_image = cv2.cvtColor(np_image, cv2.COLOR_GRAY2RGB)
    else:
        # do NOT need to do anything, or red and blue will switch
        pass
        # np_image = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)

    h, w = np_image.shape[:2]

    # Initialize virtual webcam
    if fake_cam is None:
        print(f"Initializing /dev/video0 with size {w}x{h}")
        fake_cam = pyfakewebcam.FakeWebcam('/dev/video0', w, h)

    # Push frame to virtual webcam
    fake_cam.schedule_frame(np_image)

    # Optional: display preview window
    # cv2.imshow("Industrial Cam Preview", cv2.cvtColor(np_image, cv2.COLOR_RGB2BGR))
    if cv2.waitKey(1) == 27:
        os._exit(0)

    # Re-queue the frame for continuous capture
    cam.queue_frame(frame)


def main():
    with VmbSystem.get_instance():
        cam = VmbSystem.get_instance().get_all_cameras()[0]
        with cam:
            setup_camera(cam)
            print("Streaming started. Press ESC in preview window to stop.")
            cam.start_streaming(handler=frame_handler, buffer_count=10)
            while True:
                time.sleep(1)



if __name__ == '__main__':
    main()
