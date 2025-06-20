import os
import cv2

def images_to_video(image_folder, output_video_path, fps=2):
    # 获取所有图片文件名并按顺序排序
    image_files = sorted([
        f for f in os.listdir(image_folder)
        if f.lower().endswith(('.png', '.jpg', '.jpeg'))
    ])

    if not image_files:
        print("No image files found.")
        return

    # 读取第一张图像，获取尺寸
    first_image_path = os.path.join(image_folder, image_files[0])
    frame = cv2.imread(first_image_path)
    height, width, _ = frame.shape

    # 初始化视频写入器（MP4 格式，H.264 编码）
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 或 'avc1' / 'XVID'
    video_writer = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

    for img_name in image_files:
        img_path = os.path.join(image_folder, img_name)
        img = cv2.imread(img_path)
        if img is None:
            print(f"Warning: Failed to read {img_path}")
            continue

        # 如果图像尺寸不一致，可在此自动调整
        if img.shape[1] != width or img.shape[0] != height:
            img = cv2.resize(img, (width, height))

        video_writer.write(img)

    video_writer.release()
    print(f"Video saved to: {output_video_path}")

# 示例调用
images_to_video(
    image_folder='image',                   # 图像文件夹
    output_video_path='output_video.mp4',   # 输出视频路径
    fps=2                                  # 帧率，可改为15或60等
)
