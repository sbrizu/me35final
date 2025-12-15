# Model for ball finding from universe.roboflow.com
from pupil_apriltags import Detector
import math, numpy as np
import cv2, time
import asyncio
from bleak import BleakClient, BleakScanner
import threading

################################### VARIABLES ######################
SEND_INTERVAL = 0.1
TARGET_BALL_SIZE = 85000
TAG_SIZE_TARGET = 10000
#TAG_4_TARGET = 20000
KP_DIST = .01
KP_POS = .075
KI_POS = .0001
FREQ = 4
FRAME_CENTER = 900
count = 0
size = 100 
pos = FRAME_CENTER
tag_in_view = []
total_pos_error = 0

# Motor command queue for thread-safe communication
motor_queue = None
motor_queue_lock = threading.Lock()

# Frame display queue
latest_frame = None
frame_lock = threading.Lock()

# State control
state_change_flag = None
state_lock = threading.Lock()

############################# MOTOR CONTROL FUNCTIONS ################################### 

def get_direction(num):
    if num < 0:
        return 0
    elif num >= 0:
        return 1

def calc_motion(size, target, pos):
    print(f" Object Size: {size} Center: {pos}")
    global total_pos_error
    
    dist_e = (target - size)
    pos_e = FRAME_CENTER - pos  
    speed = KP_DIST * dist_e
    
    kp_Pos_term = KP_POS * pos_e
    total_pos_error += pos_e * (1/FREQ)
    ki_Pos_term = KI_POS * total_pos_error
    turn_rate = kp_Pos_term + ki_Pos_term
    
    print(f"size: {size} speed: {speed} pos:{pos} kp:{kp_Pos_term} ki:{ki_Pos_term} turn_rate:{turn_rate}")
    
    # Limits
    turn_rate = max(-10, min(10, turn_rate))
    speed = max(-90, min(90, speed))
    
    left_motor = speed + turn_rate
    right_motor = speed - turn_rate
    
    # Convert to integers
    return int(left_motor), int(right_motor)

async def turn_45(direction):
    if direction == "right":
        await send_motor_ble(20, -20)
        await asyncio.sleep(1)
        await send_motor_ble(0, 0)
    elif direction == "left":
        await send_motor_ble(-20, 20)
        await asyncio.sleep(1)
        await send_motor_ble(0, 0)
    await asyncio.sleep(5)

################################### BLE STUFF ################################### 

# Nordic UART Service (NUS) UUIDs
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

client = None
connected = False
last_sent_time = 0

def handle_esp32_response(sender, data):
    """Handle responses from ESP32"""
    try:
        message = data.decode('utf-8').strip()
        if message:
            print(f"ESP32: {message}")
    except Exception as e:
        print(f"Decode error: {e}")
            
async def connect_ble():
    global client, connected
    device = await BleakScanner.find_device_by_name("ESP32test")
    if not device:
        print("Device not found. Make sure ESP32 is advertising!")
        return
    
    client = BleakClient(device.address)
    await client.connect()
    await asyncio.sleep(3.0)
    
    print("Connected?", client.is_connected)
    
    services = client.services
    if services:
        print("Discovered services:")
        for s in services:
            print(s.uuid)
            for c in s.characteristics:
                print("  Char:", c.uuid, c.properties)

    await client.start_notify(UART_TX_CHAR_UUID, handle_esp32_response)
    print("Notifications started!")
    connected = True

async def send_motor_ble(m1, m2):
    global connected, last_sent_time, client
    
    if not connected or not client:
        #print("not connected")
        return False
    
    # Throttle sending
    current_time = time.time()
    if current_time - last_sent_time < SEND_INTERVAL:
        return False
    
    try:
        left_dir = get_direction(m1)
        left_power = min(99, abs(m1))
        right_dir = get_direction(m2)
        right_power = min(99, abs(m2))
        
        ble_message = bytes([left_dir, left_power, right_dir, right_power])
        await client.write_gatt_char(UART_RX_CHAR_UUID, ble_message)
        
        print(f"Sent to ESP32: {ble_message}")
        last_sent_time = current_time
        return True
        
    except Exception as e:
        print(f"Send error: {e}")
        connected = False
        return False


async def flyyy_ble(val):
    global connected, last_sent_time, client
    
    if not connected or not client:
        #print("not connected")
        return False
    
    # Throttle sending
    current_time = time.time()
    if current_time - last_sent_time < SEND_INTERVAL:
        return False
    
    try:
        ble_message = bytes([val])
        await client.write_gatt_char(UART_RX_CHAR_UUID, ble_message)
        
        print(f"Sent to ESP32: {ble_message}")
        last_sent_time = current_time
        return True
        
    except Exception as e:
        print(f"Send error: {e}")
        connected = False
        return False

################################### BALL DETECTION ################################### 


        
def find_balls_V2(frame):
    cx = 0
    area = 0
    # find balls based on color
    global motor_queue
    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define blue color range in HSV
    lower_blue = np.array([35, 40, 40])
    upper_blue= np.array([85, 255, 255])
    
    # Create mask for blue regions
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # Clean up mask
    kernel = np.ones((1, 1), np.uint8)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    
    # Apply mask: only blue visible
    blue_only = cv2.bitwise_and(frame, frame, mask=blue_mask)
    
    # Find contours for blue regions
    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        biggest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest_contour)
        
        if area > 100:
            x, y, w, h = cv2.boundingRect(biggest_contour)
            cx = x + w // 2
            cy = y + h // 2
            
            # Draw bounding box and center
            cv2.rectangle(blue_only, (x, y), (x + w, y + h), (255, 255, 255), 2)
            cv2.circle(blue_only, (cx, cy), 5, (255, 255, 255), -1)
            cv2.putText(blue_only, f"({cx}, {cy})", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(blue_only, f"Area: {int(area)}", (x, y + h + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
        #send to motors       
        
        
        #return cx, area
        # Show result window
        cv2.imshow("Blue Marker Detection", blue_only)
        
        m1, m2 = calc_motion(float(area), TARGET_BALL_SIZE, float(cx))
        
        print(f"Motors: left: {m1}, right: {m2}")
        return area, cx, m1, m2
    else:
        return None, None, 0, 0
    
    


def find_tag(img):
    try:
        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img
        
        tag_detector = Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        
        tags = tag_detector.detect(
            gray,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=None
        )

        if not tags:
            print("no tags")
            return None, None, None, None

        # Pick the biggest tag
        largest_tag = max(tags, key=lambda t: cv2.contourArea(t.corners.astype("float32")))
        largest_area = cv2.contourArea(largest_tag.corners.astype("float32"))
                
        # Draw on the image
        draw_img = img.copy()
        corners = largest_tag.corners.astype(int)
        center = tuple(largest_tag.center.astype(int))

        for i in range(4):
            pt1 = tuple(corners[i])
            pt2 = tuple(corners[(i + 1) % 4])
            cv2.line(draw_img, pt1, pt2, (0, 255, 0), 2)

        cv2.circle(draw_img, center, 4, (0, 0, 255), -1)
        cv2.putText(
            draw_img,
            f"ID: {largest_tag.tag_id}",
            (center[0] + 10, center[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 0, 0),
            2
        )

        return largest_tag, largest_area, draw_img, center[0]
        
    except Exception as e:
        print(f"Error in find_tag: {e}")
        return None, None, None, None

################################### MAIN LOOP ################################### 

state = "finding_ball"

async def main():
    global state, motor_queue, motor_queue_lock, latest_frame, frame_lock, state_change_flag, state_lock
    
    # Connect BLE first
    await connect_ble()
    
    cap = cv2.VideoCapture(0)
    pipeline_running = False
    pipeline_thread = None
    pipeline = None
    
    while True:
        center = 0
        if state == "finding_ball":
            
            ret, frame = cap.read() 
            area, cx, m1, m2 = find_balls_V2(frame)
            if area > TARGET_BALL_SIZE and abs(cx - FRAME_CENTER) < 20:
                state = "picking_up_ball"
                continue
            else:
                
                await send_motor_ble(m1, m2)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # Process queued motor commands from the pipeline
                #with motor_queue_lock:
               # if motor_queue is not None:
                    #await send_motor_ble(m1, m2)
                    #motor_queue = None
                
        elif state == "picking_up_ball":
            print("------------------ Picking up ball! ------------------")
            # Uncomment these if you have the fly mechanism
            await send_motor_ble(0,0)
            await asyncio.sleep(5)
            await flyyy_ble(0)
            await asyncio.sleep(5)
            await send_motor_ble(35, 35)
            await asyncio.sleep(5)
            await send_motor_ble(-30, -30)
            await asyncio.sleep(3)
            await send_motor_ble(0, 0)
            await asyncio.sleep(3)
            
            
            # Reset and go back to finding balls or find goal
            state = "finding_goal"
            print("Moving to finding_goal state")
                
        elif state == "finding_goal":
            ret, frame = cap.read()
            tagID = None
            cv2.imshow("view", frame)
            tag, area, out, center = find_tag(frame)
            
            if tag is not None: 
                cv2.imshow("Tag Detection", out)
                tagID = tag.tag_id
                print(tagID, area)
                # Add logic here for what to do when you find the goal
            
            
            if tagID == 3:
                print("---------------------found april tag---------------------!")
                
                if area > TAG_SIZE_TARGET and abs(center - FRAME_CENTER) < 20:
                    await send_motor_ble(20, 20)
                    await flyyy_ble(1)
                    await asyncio.sleep(5)
                    await send_motor_ble(-20, -20)
                    await asyncio.sleep(2)
                    await send_motor_ble(0, 0)
                else:
                    m1, m2 = calc_motion(area, TAG_SIZE_TARGET, center) 
                    await send_motor_ble(m1, m2)
                
            else:
                print("tag 3 not seen, rotating right")
                await turn_45("right")
                continue
            
            if motor_queue is not None:
                m1, m2 = motor_queue
                await send_motor_ble(m1, m2)
                motor_queue = None
                    
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.imwrite('output.jpg', frame)
                break

        await asyncio.sleep(1/FREQ)
            
    cap.release()
    cv2.destroyAllWindows()

# Run the async main function
if __name__ == "__main__":
    asyncio.run(main())
