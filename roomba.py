from picozero import DistanceSensor
from motor_driver import MotorDriver
from time import sleep, sleep_ms, ticks_ms, ticks_diff
from machine import Pin, PWM, Timer
import micropython
import random

class DualMotorDriver:
    
    def __init__(self, right_ids: tuple, left_ids: tuple, stby_id: int) -> None:
        
        self.right_motor = MotorDriver(*right_ids)  # unzip right_pins then feed to MotorDriver
        self.left_motor = MotorDriver(*left_ids)  # unzip left_pins then feed to MotorDriver
        self.stby_pin = Pin(stby_id, Pin.OUT)
        self.enable()  # enable motor driver

    def enable(self):
        self.stby_pin.on()

    def disable(self):
        self.stby_pin.off()

    def stop(self):
        self.right_motor.stop()
        self.left_motor.stop()

    def linear_forward(self, speed=0.):
        assert 0<=speed<=1
        # Based on user's configuration, right motor FWD, left motor BWD results in forward motion.
        self.right_motor.forward(speed)
        self.left_motor.backward(speed)

    def linear_backward(self, speed=0.):
        assert 0<=speed<=1
        # Based on user's configuration, right motor BWD, left motor FWD results in backward motion.
        self.right_motor.backward(speed)
        self.left_motor.forward(speed)

    def spin_left(self, speed=0.):
        
        assert 0<=speed<=1
        # R.forward + L.forward (Arc Right based on motor configuration)
        self.right_motor.forward(speed)
        self.left_motor.forward(speed)

    def spin_right(self, speed=0.):
        
        assert 0<=speed<=1
        # R.backward + L.backward (Arc Left based on motor configuration)
        self.right_motor.backward(speed)
        self.left_motor.backward(speed)

# Global PWM duty cycle constant
DUTY_MAX = 65535

def set_leds(r_duty, g_duty, b_duty):
    
    red_led.duty_u16(r_duty)
    green_led.duty_u16(g_duty)
    blue_led.duty_u16(b_duty)

def led_blink_rgb(duration_s, freq_hz):
    # Blinks all LEDs at the same time
    period_ms = 1000 / (2 * freq_hz) # Time for one state (ON or OFF)
    start_time = ticks_ms()
    duration_ms = duration_s * 1000
    end_time = start_time + duration_ms

    print(f"Blinking at {freq_hz} Hz for {duration_s} s...")

    # Blink loop
    while ticks_diff(end_time, ticks_ms()) > 0:
        set_leds(DUTY_MAX, DUTY_MAX, DUTY_MAX)
        sleep_ms(int(period_ms))
        set_leds(0, 0, 0)
        sleep_ms(int(period_ms))

# Global Setup and HRI Variables 

# SETUP LED PINS
red_led = PWM(Pin(28))
red_led.freq(1000)
blue_led = PWM(Pin(26))
blue_led.freq(1000)
green_led = PWM(Pin(27))
green_led.freq(1000)

# SETUP SENSORS/INPUTS
button = Pin(1, Pin.IN, Pin.PULL_DOWN) # Default signal is 0 (required for check)
ds = DistanceSensor (echo=2, trigger=3)

# HRI/State Variables
STATE_PAUSE = True  # Default state after initialization
STATE_WORK = False

# Global state tracker: True = PAUSE MODE, False = WORK MODE
current_state = STATE_PAUSE
last_press = 0
debounce_ms = 200
work_mode_time = 0.0 # Accumulated WORK MODE time in seconds
button_hold_start = None

# Button Handler (IRQ)

def toggle_state(pin):
    # Interrupt handler to toggle between PAUSE and WORK modes.
    global current_state
    global last_press
    global work_mode_time
    now = ticks_ms()

    # Debounce check
    if ticks_diff(now, last_press) > debounce_ms:
        current_state = not current_state
        last_press = now

        if current_state == STATE_PAUSE:
            print("--- SWITCHED TO PAUSE MODE ---")
            dmd.stop()
        else:
            print("--- SWITCHED TO WORK MODE ---")

# Attach the IRQ handler
button.irq(trigger=Pin.IRQ_RISING, handler=toggle_state)

if __name__=="__main__":
    # Motor setup
    dmd = DualMotorDriver(right_ids=(7, 9, 8), left_ids=(15, 13, 14), stby_id=12)
    dmd.stop()

    # Constants
    BASE_SPEED = 0.6
    AVOIDANCE_DISTANCE_M = 0.20 # Distance to avoid
    BACKUP_TIME_S = 0.5         # Time to reverse
    TURN_TIME_S = 0.8          # Time to spin (approx. 180 degrees)

    # Check 1: Button's GPIO pin is receiving correct default signal (0 for PULL_DOWN)
    button_check_ok = (button.value() == 0)

    # Check 2: Ultrasonic sensor is receiving non-zero distance measuring
    distance_check_ok = False
    try:
        # Read distance a few times to ensure a valid read
        for _ in range(5):
            distance = ds.distance
            if distance is not None and distance > 0.0:
                distance_check_ok = True
                break
            sleep_ms(50)
    except Exception as e:
        print(f"Distance sensor read error: {e}")

    # Blink LEDs only if both conditions are met 
    if button_check_ok and distance_check_ok:
        print("System Check Passed (Button 0, Distance > 0).")
        led_blink_rgb(duration_s=2, freq_hz=5) # 5 Hz, 2 seconds
    else:
        print(f"System Check FAILED: Button 0={button_check_ok}, Distance > 0={distance_check_ok}")

    # Robot enters PAUSE MODE after this step
    current_state = STATE_PAUSE
    dmd.stop()
    print("Entering PAUSE MODE.")
    set_leds(0, 0, 0) # Ensure all off initially

    last_tick = ticks_ms()

    try:
        blink_red_on = False
        red_blink_timer = 0
        fade_direction = 1 # 1 for in, -1 for out
        fade_duty = 0
        fade_step = int(DUTY_MAX / 50) # Step size for a smoother fade over 

        # Main Loop
        while True:
            now = ticks_ms()
            elapsed_ms = ticks_diff(now, last_tick)

            # HRI: Button Hold Reset (3 seconds)
            if button.value() == 1:
                if button_hold_start is None:
                    button_hold_start = now
                elif ticks_diff(now, button_hold_start) >= 3000:
                    print("Button held for 3 seconds — triggering reset.")
                    dmd.stop()
                    dmd.disable()
                    reset() # Machine reset
            else:
                button_hold_start = None

            # HRI: Low Battery Simulation
            current_speed = BASE_SPEED # Default speed

            if not current_state:
                # Only accumulate time in WORK MODE
                work_mode_time += elapsed_ms / 1000.0
                
                if work_mode_time > 45.0:
                    # Low battery: 50% speed 
                    current_speed = BASE_SPEED * 0.5

            last_tick = now

            # Determine the LED color to use (GREEN or BLUE)
            if work_mode_time > 45.0:
                # Low battery: Substitute GREEN with BLUE 
                work_led = blue_led
                other_led = green_led
            else:
                work_led = green_led
                other_led = blue_led

            # STATE MACHINE

            if current_state == STATE_PAUSE:
                # PAUSE MODE
                dmd.stop() # Robot stop moving 

                # GREEN/BLUE LED fades in and fades out at 1 Hz
                fade_duty += fade_direction * fade_step
                
                if fade_duty >= DUTY_MAX:
                    fade_direction = -1 # Start fading out
                    fade_duty = DUTY_MAX # Clamp max
                elif fade_duty <= 0:
                    fade_direction = 1 # Start fading in
                    fade_duty = 0 # Clamp min

                work_led.duty_u16(int(fade_duty))
                other_led.duty_u16(0)
                red_led.duty_u16(0)
                
                sleep_ms(10) # Small delay for fade smoothness

            else:
                
                # Check for critical battery termination
                if work_mode_time > 60.0:
                    print("RED LED blinked for 5 seconds — triggering reset.")
                    # The loop will break and the cleanup/reset code will run below
                    break

                # Critical battery simulation: Blink RED at 10 Hz 
                if work_mode_time > 55.0:
                    red_blink_timer += elapsed_ms
                    red_blink_period_ms = 50 # 10 Hz blink rate

                    if red_blink_timer >= red_blink_period_ms:
                        blink_red_on = not blink_red_on
                        red_blink_timer = 0

                    red_led.duty_u16(DUTY_MAX if blink_red_on else 0)
                    work_led.duty_u16(0)
                    other_led.duty_u16(0)
                
                # Normal/Low battery indication
                elif work_mode_time > 45.0:
                    # BLUE LED stays constantly on (Low battery simulation)
                    work_led.duty_u16(DUTY_MAX)
                    red_led.duty_u16(0)
                    other_led.duty_u16(0)
                else:
                    # GREEN LED stays constantly on (Normal work mode) 
                    work_led.duty_u16(DUTY_MAX)
                    red_led.duty_u16(0)
                    other_led.duty_u16(0)

                # 2. Motor and Sensor Logic 
                distance = ds.distance

                # Obstacle Avoidance Logic: Back up, turn, then continue forward
                if distance is not None and distance < AVOIDANCE_DISTANCE_M:
                    print(f"Obstacle at {distance:.2f} m. Avoidance sequence...")

                    # A. Stop
                    dmd.stop()
                    sleep(0.1)

                    # B. Back Up
                    dmd.linear_backward(current_speed)
                    sleep(BACKUP_TIME_S)
                    dmd.stop()

                    # C. Turn (Randomize turn direction to prevent getting stuck)
                    if random.getrandbits(1) == 1:
                        dmd.spin_left(current_speed)
                    else:
                        dmd.spin_right(current_speed)

                    sleep(TURN_TIME_S * random.uniform(0.8, 1.2)) # Slight randomization
                    dmd.stop()

                # Drive Forward (Robot start moving without hitting the wall)
                dmd.linear_forward(current_speed)
                sleep_ms(10) # Small loop delay

    except KeyboardInterrupt:
        print("\nProgram interrupted.")
        pass

    # Termination Cleanup 
    dmd.stop()
    print("Motors stopped.")
    set_leds(0, 0, 0)
    sleep(0.1)
    dmd.disable()
    print("Motor driver disabled. System offline.")

