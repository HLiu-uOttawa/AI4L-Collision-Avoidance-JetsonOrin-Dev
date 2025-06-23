import cv2
import os
from natsort import natsorted

# Parameters 
image_folder = 'May_5_Experiments/Experiment_A/image'       # Replace with your image folder
output_video = '2025_06_03_Exp1_video.mp4'   # Output file name
fps = 6                   # Frames per second

# Get list of image files
images = [img for img in os.listdir(image_folder) if img.lower().endswith(('.png', '.jpg', '.jpeg'))]
images = natsorted(images)  # Natural sort (e.g., img1, img2, ..., img10)

if not images:
    raise ValueError("No images found in the folder.")

# Read first image to get dimensions
first_image_path = os.path.join(image_folder, images[0])
frame = cv2.imread(first_image_path)
height, width, layers = frame.shape

# Define video writer
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'XVID' or 'avc1' for other formats
video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

# Add images to video
for image in images:
    img_path = os.path.join(image_folder, image)
    frame = cv2.imread(img_path)
    if frame is None:
        print(f"Warning: Couldn't read {img_path}")
        continue
    frame_resized = cv2.resize(frame, (width, height))  # Optional resize
    video.write(frame_resized)

# Release the video writer
video.release()
print(f"Video saved as {output_video}")