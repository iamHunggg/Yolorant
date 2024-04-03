
# Yolorant

A neural network aim assist that uses real-time object detection accelerated with CUDA on Nvidia GPUs and Arduino (detected)

# About
- Yolorant can be used with all types of FPS games by change the weights file (bestnew.pt or bestnew.engine).
- The basis of player detection in Yolorant is the YOLOv8 architecture written in PyTorch.
- The strength of this project lies in its avoidance of memory hacking, making it less prone to detection and significantly more accurate compared to Colorbot.

# Requirements
1. Hardware
- Arduino Leonardo or Arduino Uno R4 or similar boards with HID support
- Usb host shield.
2. Software
- Install a version of [Python](https://www.python.org/downloads/) 3.9 or later and [Arduino](https://www.arduino.cc/en/software) 1.8 or later.

# Setup
1. Hardware
- Solder onto the 3.3v and 5v points as depicted below:
![thumbnail](https://github.com/iamHungdz/Valorant_Yolov8/assets/113734844/d6f7ee98-1281-4bba-8e07-939f2d6e9ad8)
- Attach the USB host shield to your board.
![thumbnail](https://github.com/iamHungdz/Valorant_Yolov8/assets/113734844/fe4eb6bf-3205-4759-8d09-19071e39a223)

2. Software
- Upload code to your board.
- Navigate to the root directory. Use the package manager pip to install the necessary dependencies.
```
pip install -r requirements.txt
```
    
# Usage
- Launch the game and run main.py.

# Preview
- https://www.youtube.com/watch?v=KGw_yPY1MdY
- https://www.youtube.com/watch?v=xrXGrTPqDdY
  (I'm not too good at this game since I don't play it much, so my skills are pretty trash :) )
# Issues
- The process of fetching data and transmitting it to Arduino, coupled with mouse movement methodology (String read), incurs sluggishness, thereby harboring several limitations.
