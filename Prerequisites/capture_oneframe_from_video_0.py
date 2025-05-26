import cv2

# Open the video device
cap = cv2.VideoCapture('/dev/video0')

# Optional: set resolution if needed
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Check if the device opened correctly
if not cap.isOpened():
    print("Error: Cannot open /dev/video0")
else:
    # Read one frame
    ret, frame = cap.read()
    if ret:
        # Save or show the frame
        cv2.imwrite('frame.jpg', frame)
        print("Frame captured and saved as frame.jpg")
    else:
        print("Error: Failed to capture frame")

# Release the device
cap.release()
