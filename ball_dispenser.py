'''''
Ball Dispenser 
ME 35 Final: Football
Written by Sol Brizuela
Sample code from Professor Milan Dahal
'''''


from machine import PWM, Pin, time_pulse_us
import time

# --------- SERVO SETUP ---------
servo1 = PWM(Pin(19), freq=50, duty_u16=0)   # Door 1
servo2 = PWM(Pin(18), freq=50, duty_u16=0)   # Door 2

CLOSED = int(1500 * 1000)
OPEN_DOOR1 = int(2500 * 1000)   # Adjust if needed
OPEN_DOOR2 = int(500 * 1000)    # Adjust if needed

# Start with both doors closed
servo1.duty_ns(CLOSED)
servo2.duty_ns(CLOSED)
time.sleep(1)

# --------- ULTRASONIC SETUP (Grove, single SIG pin) ---------
SIG = Pin(32, Pin.OUT)

def get_distance_cm():
    # Trigger pulse
    SIG.init(Pin.OUT)
    SIG.value(0)
    time.sleep_us(2)
    SIG.value(1)
    time.sleep_us(10)
    SIG.value(0)

    # Listen for echo
    SIG.init(Pin.IN)
    duration = time_pulse_us(SIG, 1, 30000)   # timeout 30ms

    if duration < 0:
        return None

    distance_cm = duration / 58.0
    return distance_cm

# --------- DOOR CYCLE ---------
def run_door_cycle():
    # Door 1
    print("Door 1 OPEN")
    servo1.duty_ns(OPEN_DOOR1)
    time.sleep(1)

    print("Door 1 CLOSED")
    servo1.duty_ns(CLOSED)
    time.sleep(2)   # wait 2 seconds before Door 2

    # Door 2
    print("Door 2 OPEN")
    servo2.duty_ns(OPEN_DOOR2)
    time.sleep(1)

    print("Door 2 CLOSED")
    servo2.duty_ns(CLOSED)

# --------- MAIN LOOP ---------
THRESHOLD_CM = 10

while True:
    dist = get_distance_cm()
    if dist is None:
        print("No echo")
    else:
        print("Distance:", round(dist, 2), "cm")

        if dist < THRESHOLD_CM:
            print("Object detected closer than", THRESHOLD_CM, "cm -> running door cycle")
            run_door_cycle()
            # small cooldown so it doesn't retrigger instantly
            time.sleep(10)

    time.sleep(0.1)
