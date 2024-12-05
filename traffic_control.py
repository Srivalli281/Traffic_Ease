import threading
import time
import cv2
from ultralytics import YOLO
from datetime import datetime

global timeseconds

# Load YOLO models
vehicle_model = YOLO('/content/drive/MyDrive/Traffic Ease/Models/Pretrained Model/yolov8n.pt')
ambulance_model = YOLO('/content/drive/MyDrive/Traffic Ease/Models/Ambulance Model/train/weights/best.pt')


# Initialize global variables
#video_paths = []
vehicle_counts = [0] * 4
lane_times = [0] * 4
current_green_lane = None
lane_videos = []
ambulance_detected_in_current_lane = False
recently_visited_lanes = []

def set_video_paths(paths):
    global lane_videos
    lane_videos = [cv2.VideoCapture(video) for video in paths]

#  Capture a frame at a specific second from the video.
def capture_frame(video,second):
    video.set(cv2.CAP_PROP_POS_MSEC, second * 1000)
    ret, frame = video.read()
    if ret:
      return frame
    else:
      None

  
#Count vehicles using YOLO Pretrained model
def count_vehicles(frame, model):
    results = model(frame)
    vehicle_classes = {1, 2, 3, 5, 7}  
    boxes = results[0].boxes 
    vehicle_count = sum(1 for box in boxes if int(box.cls) in vehicle_classes)
    return vehicle_count

#detect ambulance using custom ambulance model
def detect_ambulance1(frame,model,confidencethreshold=0.4):
    results = model(frame)
    amb_class = {0}
    boxes = results[0].boxes  
    if not boxes:
        return False
    for box in boxes:
      if int(box.cls) in amb_class and box.conf >= confidencethreshold:
        return True
      else:
        return False

#Continuously check for ambulances every 10 seconds while the green light timer is running.
def check_ambulance_concurrently(vehicle_counts):
    global current_green_lane, lane_times
    checktime =1 # Initialize check time
    while lane_times[current_green_lane] > 0:
        if checktime % 10 == 0:  
            current_frames = [capture_frame(video, timeseconds) for video in lane_videos]
            ambulance_detected = [
                detect_ambulance1(frame, ambulance_model) if frame is not None else False
                for frame in current_frames
            ]
            print(ambulance_detected)
            amb_flag = False
            if True in ambulance_detected:
                amb_flag = True
                ambulance_lane = ambulance_detected.index(True)
                if ambulance_lane is not None and ambulance_lane != current_green_lane:
                    print(f"Ambulance detected in lane {ambulance_lane}. Switching to this lane immediately.")
                    for i in range (0,3):
                       print(f"🚦🟡 Yellow signal for lane {current_green_lane}. Preparing to switch.")
                    stop_green=True
                    switch_to_lane(ambulance_lane, vehicle_counts, amb_flag)
                    break

        time.sleep(1) 
        checktime += 1  


def display_lane_lights(current_green_lane, transition_lane=None):
    lights = []
    for i in range(4):
        if i == current_green_lane:
            lights.append("🚦🟢") 
        elif i == transition_lane:
            lights.append("🚦🟡")  
        else:
            lights.append("🚦🔴")

    print("\nCurrent Lane Status:")
    for i, light in enumerate(lights):
        print(f"Lane {i}: {light}")

#Handling switching lanes
def switch_to_lane(new_green_lane, vehicle_counts, amb_flag=False):
    global current_green_lane, recently_visited_lanes
    current_green_lane = new_green_lane
    print(f"🚦🟢 Green light allocated to lane {new_green_lane}.")
    display_lane_lights(current_green_lane)
    if amb_flag:
        lane_times[current_green_lane] = 20
    else:
        average_count = sum(vehicle_counts) / len(vehicle_counts)
        if average_count > 50:
            lane_times[current_green_lane] = 20
        elif average_count > 60:
            lane_times[current_green_lane] = 20
        elif average_count > 45:
            lane_times[current_green_lane] = 20
        elif average_count > 20:
            lane_times[current_green_lane] = 20
        else:
            lane_times[current_green_lane] = 20

    recently_visited_lanes.append(current_green_lane)
    if len(recently_visited_lanes) == 4: 
        recently_visited_lanes.clear()

#Main logic to manage traffic control based on vehicle counts and ambulance detection.
def main_traffic_control():
    global current_green_lane, lane_times, timeseconds,stop_green
    stop_green=False
    timeseconds = 1
    while not stop_green: 
        # Step 1: Capture initial frames and calculate vehicle counts
        current_frames = [capture_frame(video, timeseconds) for video in lane_videos]
        hasnone=any(x is None for x in current_frames)
        if hasnone:
          stop_green=True
          break
        current_frames_path = []
        """
        for i, frame in enumerate(current_frames):
            if frame is not None:
                # Generate a unique filename using the timestamp and lane index
                filename = f"frame_lane_{i+1}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"

                # Save the frame to disk
                cv2.imwrite(filename, frame)

                # Append the saved path to the list
                current_frames_path.append(filename)
      """
        vehicle_counts[:] = [
            count_vehicles(frame, vehicle_model) if frame is not None else 0
            for frame in current_frames
        ]

        # Step 2: Handle ambulance detection for each lane
        ambulance_detected = [
            detect_ambulance1(frame, ambulance_model) if frame is not None else False
            for frame in current_frames
        ]
        print(ambulance_detected)
        checkflag=False

        # Step 3: Determine initial lane to allocate green light
        amb_flag = False

        if True in ambulance_detected:
            # Prioritize the lane with an ambulance
            checkflag=False
            amb_flag = True
            ambulance_lane = ambulance_detected.index(True)
            print(f"🚨 Ambulance detected in lane {ambulance_lane}. Prioritizing this lane.")

            switch_to_lane(ambulance_lane, vehicle_counts, amb_flag)
        else:
            checkflag=True
            # Allocate green light to the lane with the highest vehicle count
            sorted_vehicle_counts = sorted(enumerate(vehicle_counts), key=lambda x: x[1], reverse=True)
            for lane, _ in sorted_vehicle_counts:
                if lane not in recently_visited_lanes:
                    print(f"No ambulance detected. 🚦🟢 Prioritizing lane {lane} with the highest vehicle count.")
                    switch_to_lane(lane, vehicle_counts, amb_flag)
                    break

       
        if(checkflag==True):
          ambulance_thread = threading.Thread(target=check_ambulance_concurrently, args=(vehicle_counts,), daemon=True)
          ambulance_thread.start()
        # Step 4: Start green light timer and concurrent ambulance detection
        while lane_times[current_green_lane] >= 0:
            if(lane_times[current_green_lane]==0):
              for i in range(0,3):

                print(f"🚦🟡 Yellow signal for lane {current_green_lane}. Preparing to switch.")
                time.sleep(1)
              print(f"🚦🔴 Green time for lane {current_green_lane} completed. Reevaluating traffic.")
            elif (lane_times[current_green_lane]!=0):
              print(f"🚦🟢 Lane {current_green_lane} green time remaining: {lane_times[current_green_lane]} seconds.")
            time.sleep(1)  
            timeseconds += 1
            lane_times[current_green_lane] -= 1

        # Wait for ambulance thread to complete if it's still running
        if(checkflag==True):
          ambulance_thread.join()
        if stop_green:
            print("Stopping traffic control due to stop signal.")
            break
    print("Traffic control system has been stopped.")

def start_traffic_control():
   #global stop_green
   lane_threads = []
   for i, video in enumerate(lane_videos):
       t = threading.Thread()
       t.start()
       lane_threads.append(t)
   try:
     main_traffic_control()
   except KeyboardInterrupt:
        print("KeyboardInterrupt received. Stopping...")
        global stop_green
        stop_green = True
   finally:
        print("Cleaning up resources...")








