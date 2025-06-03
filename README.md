# AI4L-Collision-Avoidance-JetsonOrin
This project is for Collision avoidance, which can be run directly on Jetson Orin NX without docker container.
JetPack 6.2 / L4T 36.4.3 / kernel 5.15.148-tegra

1. Setup .venv on jetson nano nx 

2. Setup .venv on Windows
   3. Python version is : 3.10.12
      https://www.python.org/downloads/windows/
      3. 
   python -m venv .venv
   
   .\.venv\Scripts\activate
3. 
   python.exe -m pip install --upgrade pip
4. 
   pip3 install ultralytics==8.3.143 plotly==5.22.0 stonesoup==1.2 scikit-learn==1.3.2 pyserial==3.5 pynmea2==1.19.0
   pip3 install qrcode[pil] pyzbar

   python .\tracking.py --skip-radar --skip-video --skip-imu --skip-gps --skip-tracking


# Hardware:
## VK-162 G-Mouse USB GPS Dongle
[GitHub - VK-16_GPS](https://github.com/AbdullahJirjees/VK-16_GPS/tree/main)
[GNSS evaluation software for Windows](https://www.u-blox.com/en/product/u-center)