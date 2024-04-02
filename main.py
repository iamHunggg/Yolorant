from ultralytics import YOLO
import dxcam
import serial
import numpy as np
import winsound
import win32api
import time
import threading
import os
import json
from scipy.spatial import KDTree


with open('settings.json', 'r') as file:
    config = json.load(file)

sens = config['sens']
COM = config['COM']
try:
    Serial = serial.Serial(COM,1000000,timeout = 0) 
except:
    raise Exception("The arduino is not connected to PC or the Arduino's COM port is wrong.")

DISPLAY_SCALE = config['DISPLAY_SCALE']
fovX = config['fovX']
fovY = config['fovY']

scale = [fovX, fovY] #range detect

# Calculate the center of the screenshot
screenshot_center = [scale[0]/2 , scale[1]/2]
camera            = dxcam.create(device_idx=0,output_idx=0,output_color="BGR") 
region            = (int((DISPLAY_SCALE[0]-scale[0])/2),int((DISPLAY_SCALE[1]-scale[1])/2),
                    int((DISPLAY_SCALE[0]+scale[0])/2),int((DISPLAY_SCALE[1]+scale[1])/2))#region capture screen
model             = YOLO("bestnew.engine")



Triggerbot=False #Triggerbot state

Triggerbot_key=win32api.VkKeyScan(config['TriggerbotKey'])

Aimbot=False     #Aimbot state
Aimbot_key=win32api.VkKeyScan(config['AimbotKey'])

Aim_assist=False  #Aim_assist state
Aim_assist_key=win32api.VkKeyScan(config['AimassistKey'])

Flick=False   #Flick state
Flick_cd=True
Flick_key=win32api.VkKeyScan(config['FlickKey'])


Flick_delay = config['Flick_delay']
last_flick_time = 0

Shot_key=win32api.VkKeyScan(config['FireKey'])

running=False

ctsx = (1.07437623) * ((sens) ** (-0.9936827126))/(DISPLAY_SCALE[0])*(1920)
ctsy = (1.07437623) * ((sens) ** (-0.9936827126))/(DISPLAY_SCALE[1])*(1080)
smooth=1.2

if config['target']=='head': #'head' or 'body'
    target_enemy=1
else:
    target_enemy=0

def ProcessKeyPress():
    global Aim_assist,Triggerbot,Aimbot,Flick,running,fovX,fovY,region
    running=True
    if win32api.GetAsyncKeyState(Aimbot_key)&0x8000 > 0: #Aimbot
        Aim_assist = False
        Flick      = False
        Aimbot     = not Aimbot
        if Aimbot:
            winsound.PlaySound(os.path.join(os.path.dirname(__file__), 'sound', 'aimboton.wav'), winsound.SND_FILENAME | winsound.SND_ASYNC)
        else:
            Triggerbot=False
            winsound.PlaySound(os.path.join(os.path.dirname(__file__), 'sound', 'aimbotoff.wav'), winsound.SND_FILENAME | winsound.SND_ASYNC)
        time.sleep(0.2)
    if win32api.GetAsyncKeyState(Aim_assist_key)&0x8000 > 0: #Aim_assist
        Aim_assist = not Aim_assist
        Triggerbot = False
        Aimbot     = False  
        Flick      = False
        if Aim_assist:
            winsound.PlaySound(os.path.join(os.path.dirname(__file__), 'sound', 'aimassiston.wav'), winsound.SND_FILENAME | winsound.SND_ASYNC)
        else:
            winsound.PlaySound(os.path.join(os.path.dirname(__file__), 'sound', 'aimassistoff.wav'), winsound.SND_FILENAME | winsound.SND_ASYNC)
        time.sleep(0.2)
    if win32api.GetAsyncKeyState(Triggerbot_key)&0x8000 > 0: #Triggerbot
        Aim_assist = False
        Triggerbot = not Triggerbot
        Flick      = False
        if Triggerbot:
            winsound.PlaySound(os.path.join(os.path.dirname(__file__), 'sound', 'triggerboton.wav'), winsound.SND_FILENAME | winsound.SND_ASYNC)
        else:
            winsound.PlaySound(os.path.join(os.path.dirname(__file__), 'sound', 'triggerbotoff.wav'), winsound.SND_FILENAME | winsound.SND_ASYNC)
        time.sleep(0.2)
    if win32api.GetAsyncKeyState(Flick_key)&0x8000 > 0: #Flick
        Aim_assist = False
        Triggerbot = False
        Aimbot     = False
        Flick      = not Flick
        
        if Flick:
            winsound.PlaySound(os.path.join(os.path.dirname(__file__), 'sound', 'flickon.wav'), winsound.SND_FILENAME | winsound.SND_ASYNC)
        else:
            winsound.PlaySound(os.path.join(os.path.dirname(__file__), 'sound', 'flickoff.wav'), winsound.SND_FILENAME | winsound.SND_ASYNC)
        time.sleep(0.2)
    if win32api.GetAsyncKeyState(0x26)&0x8000 > 0: #FovY++
        fovY+=10
        print(f"fovY{fovY}")
        config['fovX'] = fovY
        with open('settings.json', 'w') as file:
            json.dump(config, file, indent=2)
        region         = (int((DISPLAY_SCALE[0]-scale[0])/2),int((DISPLAY_SCALE[1]-fovY)/2),
                    int((DISPLAY_SCALE[0]+scale[0])/2),int((DISPLAY_SCALE[1]+fovY)/2))#region capture screen
        
        time.sleep(0.2)
    if  win32api.GetAsyncKeyState(0x26)&0x8000 > 0: #FovY--
        fovY-=10
        print(f"fovY{fovY}")
        config['fovX'] = fovY
        with open('settings.json', 'w') as file:
            json.dump(config, file, indent=2)
        region         = (int((DISPLAY_SCALE[0]-scale[0])/2),int((DISPLAY_SCALE[1]-fovY)/2),
                    int((DISPLAY_SCALE[0]+scale[0])/2),int((DISPLAY_SCALE[1]+fovY)/2))#region capture screen
        
        time.sleep(0.2)
    if  win32api.GetAsyncKeyState(0x27)&0x8000 > 0: #FovX++
        fovX+=10
        print(f"fovY{fovX}")
        config['fovX'] = fovX
        with open('settings.json', 'w') as file:
            json.dump(config, file, indent=2)
        region          = (int((DISPLAY_SCALE[0]-fovX)/2),int((DISPLAY_SCALE[1]-scale[1])/2),
                    int((DISPLAY_SCALE[0]+fovX)/2),int((DISPLAY_SCALE[1]+scale[1])/2))#region capture screen
        time.sleep(0.2)
    if  win32api.GetAsyncKeyState(0x25)&0x8000 > 0: #FovX--
        fovX-=10
        print(f"fovY{fovX}")
        config['fovX'] = fovX
        with open('settings.json', 'w') as file:
            json.dump(config, file, indent=2)
        region            = (int((DISPLAY_SCALE[0]-fovX)/2),int((DISPLAY_SCALE[1]-scale[1])/2),
                    int((DISPLAY_SCALE[0]+fovX)/2),int((DISPLAY_SCALE[1]+scale[1])/2))#region capture screen
        time.sleep(0.2) 
    
    running=False

while True:
    if running==False:
        thread1 = threading.Thread(target=ProcessKeyPress)
        thread1.start()        
    screenshot = camera.grab(region)
    if screenshot is None: continue

    closest_part_distance = np.inf
    closest_part = -1

    results =model(screenshot,classes=target_enemy,conf=0.2) #class 1 is enemy_head

    boxes=results[0].boxes
    # Convert the bounding box coordinates to a set of points
    points = [[(box.xyxy[0].cpu().numpy()[2] + box.xyxy[0].cpu().numpy()[0]) / 2,
        (box.xyxy[0].cpu().numpy()[3] + box.xyxy[0].cpu().numpy()[1]) / 2] for box in boxes]

    # Reshape the points array to have two dimensions
    points = np.array(points)
    points = points.reshape((-1, 2))

    # Build a k-d tree from the set of points
    tree = KDTree(points)

    # Find the nearest neighbor to the center of the screen
    center_display = np.array(screenshot_center)
    dist, index = tree.query(center_display)

    # Check if any objects were detected
    if len(boxes) > 0:
        # Get the bounding box coordinates of the nearest neighbor
        xmin, ymin, xmax, ymax = boxes[index].xyxy[0].cpu().numpy()

         #triggerbot (mouse click)
        if Triggerbot==True and screenshot_center[0] in range(int(xmin),int(xmax)) and screenshot_center[1] in range(int(ymin),int(ymax)):
            win32api.keybd_event(Shot_key, 0, 0, 0)  # Press key
            win32api.keybd_event(Shot_key, 0, 2, 0)  # Release key

        #aimbot (move mouse to move_distance,smooth move)
        elif Aimbot :
            center_x = (xmax + xmin) / 2
            center_y = (ymax + ymin) / 2
            #calculate the distance needed to move the mouse
            move_distance = [(center_x - screenshot_center[0])*smooth, (center_y - screenshot_center[1])*smooth]
            data_str = f"a{move_distance[0]},{move_distance[1]}\n"
            Serial.write(data_str.encode())
            
        #flick (move the mouse to move_distance,click)
        elif Flick:
            # Calculate the time elapsed since the last "flick" operation
            current_time = time.time()
            time_elapsed = current_time - last_flick_time
            center_x = (xmax + xmin) / 2
            center_y = (ymax + ymin) / 2
            
            # Check if its been at least Flick_delay(ms) since the last "flick"
            if time_elapsed >= Flick_delay : #and np.linalg.norm((center_x,center_y)-center_display)<20

                # calculate the distance needed to move the mouse
                move_distance = [((center_x - screenshot_center[0])*ctsx),((center_y - screenshot_center[1])*ctsy)]
                data_str = f"f{move_distance[0]},{move_distance[1]}\n"
                # Send the data_str and update the last_flick_time
                Serial.write(data_str.encode())
                last_flick_time = current_time  

        #aim_assist (When clicked, move the mouse to move_distance(like aimbot but need click))
        elif Aim_assist and win32api.GetAsyncKeyState(0x01)&0x8000 > 0:
            center_x = (xmax + xmin) / 2
            center_y = (ymax + ymin) / 2
            #calculate the distance needed to move the mouse
            move_distance = [(center_x - screenshot_center[0])*smooth,(center_y - screenshot_center[1])*smooth]
            data_str = f"a{move_distance[0]},{move_distance[1]}\n"
            Serial.write(data_str.encode())
