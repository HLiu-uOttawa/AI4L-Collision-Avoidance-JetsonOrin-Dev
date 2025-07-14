import os
import cv2

def images_to_video(image_folder, output_video_path, fps=2):
    # Get all image filenames in the folder and sort them in order
    image_files = sorted([
        f for f in os.listdir(image_folder)
        if f.lower().endswith(('.png', '.jpg', '.jpeg'))
    ])

    if not image_files:
        print("No image files found.")
        return

    # Read the first image to get frame dimensions
    first_image_path = os.path.join(image_folder, image_files[0])
    frame = cv2.imread(first_image_path)
    height, width, _ = frame.shape

    # Initialize the video writer (MP4 format, H.264 or similar codec)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Alternatives: 'avc1', 'XVID'
    video_writer = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

    # Loop through all image files and write them to the video
    for img_name in image_files:
        img_path = os.path.join(image_folder, img_name)
        img = cv2.imread(img_path)
        if img is None:
            print(f"Warning: Failed to read {img_path}")
            continue

        # Resize the image if it does not match the video frame size
        if img.shape[1] != width or img.shape[0] != height:
            img = cv2.resize(img, (width, height))

        # Write the image frame to the video
        video_writer.write(img)

    # Release the video writer to finalize the file
    video_writer.release()
    print(f"Video saved to: {output_video_path}")

# Example usage
images_to_video(
    image_folder='image',                   # Folder containing image files
    output_video_path='output_video.mp4',   # Output video file path
    fps=2                                   # Frames per second (adjustable)
)
