import os
from PIL import Image

def resize_images_to_other_folder(src_folder, dst_folder):
    os.makedirs(dst_folder, exist_ok=True)

    for filename in os.listdir(src_folder):
        if filename.lower().endswith('.png'):
            src_path = os.path.join(src_folder, filename)
            dst_path = os.path.join(dst_folder, filename)
            print(f"Resizing: {src_path} -> {dst_path}")

            img = Image.open(src_path)
            new_size = (img.width // 3, img.height // 3)
            resized_img = img.resize(new_size, Image.LANCZOS)

            resized_img.save(dst_path)

resize_images_to_other_folder(
    src_folder='./video',
    dst_folder='./video_resized'
)
