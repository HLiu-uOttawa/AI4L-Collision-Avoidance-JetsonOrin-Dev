# Set up the USB3.0 Camera
# 0. 
# wget https://downloads.alliedvision.com/VimbaX/VimbaX_Setup-2025-1-Linux_ARM64.tar.gz  
# sudo apt update  
# sudo apt install v4l2loopback-dkms  
# pip3 install pyfakewebcam  

# 0. Activate .venv
# Enter the folder: nvidia@ubuntu:~/workspace/AI4L-Collision-Avoidance-JetsonOrin-Dev  
# $ source .venv/bin/activate  

# 1. VirtualCam
# Enter the folder: nvidia@ubuntu:~/workspace/AI4L-Collision-Avoidance-JetsonOrin-Dev/Prerequisites  
# sudo modprobe v4l2loopback devices=1 video_nr=0 card_label="VirtualCam"  

#
ll /dev/video*  
#
nohup python3 stream_to_virtual_cam.py > output.log 2>&1 &  
#
