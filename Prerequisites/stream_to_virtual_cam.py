import os
import cv2
import time
import numpy as np
import pyfakewebcam
from vmbpy import *

# Global variables
fake_cam = None
last_time = 0
frame_interval = 1 / 30  # Limit frame rate to 30 FPS

def add_qrcode_to_image(np_image, text, qr_size=60):
    import qrcode
    from PIL import Image
    import numpy as np

    # 生成 QRCode
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=1,
        border=1,
    )
    qr.add_data(text)
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white").convert("RGB")
    qr_np = np.array(img.resize((qr_size, qr_size)))

    # paste QR
    h, w = np_image.shape[:2]
    x_offset = w - qr_size - 5  # 5px
    y_offset = h - qr_size - 5  # 5px
    np_image[y_offset:y_offset+qr_size, x_offset:x_offset+qr_size] = qr_np

    return np_image

def setup_camera(cam: Camera):
    with cam:
        try:
            cam.BinningHorizontal.set(3)
            cam.BinningVertical.set(3)
        except:
            print("Binning to 2x2: failed")

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
            cam.BalanceWhiteAuto.set('Continuous')  # Continuous
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

    # === Add high-precision timestamp (ms) header to image ===
    # now = time.time()
    # timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
    # ms = int((now - int(now)) * 1000)
    # full_timestamp = f"{timestamp}.{ms:03d}"  # e.g., 2025-05-28 15:47:32.123
    #
    # cv2.putText(np_image,
    #             f"Time: {full_timestamp}",
    #             (10, 30),  # position
    #             cv2.FONT_HERSHEY_SIMPLEX,
    #             0.8,               # font scale
    #             (255, 0, 0),       # color (RGB)
    #             2,                 # thickness
    #             cv2.LINE_AA)

    # === Add high-precision timestamp (ms) header to image ===
    # Timestamp
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
    ms = int((now - int(now)) * 1000)
    full_timestamp = f"{timestamp}.{ms:03d}"

    # Paste QRCode！
    np_image = add_qrcode_to_image(np_image, full_timestamp, qr_size=60)


    # Initialize virtual webcam
    if fake_cam is None:
        print(f"Initializing /dev/video0 with size {w}x{h}")
        fake_cam = pyfakewebcam.FakeWebcam('/dev/video0', w, h)

    # Push frame to virtual webcam
    fake_cam.schedule_frame(np_image)
    print(f"Frame pushed:{full_timestamp}")

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
