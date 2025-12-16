from XRPLib.defaults import *
import time
import json
import math
import random
from machine import UART, Pin

# ==========================================
# CONSTANTS
# ==========================================

# ------------------------------------------
# 1. System & Hardware
# ------------------------------------------
UART_BAUDRATE = 115200      # Communication speed with Coral Micro
UART_TIMEOUT = 200          # Read timeout in milliseconds
RESTART_WAIT = 2.0          # Time to wait for Coral Micro to boot (seconds)
SERVO_HOME = 180.0          # Initial servo position (stowed/up)

# IDs (Coral Model)
ID_BASKET = 2
ID_PLATE = 3
ID_ARROW = 1

# ------------------------------------------
# 2. State Machine
# ------------------------------------------
STATE_FIND_LINE = 0
STATE_APPROACH_PLATE = 1
STATE_PARK_PLATE = 2
STATE_RETURN_TO_LINE = 3
STATE_REALIGN_LINE = 4
STATE_FOLLOW_TO_END = 5
STATE_EXPLORE = 6
STATE_FINE_TUNE_BASKET = 7
STATE_PICKUP_BASKET = 8
STATE_BASKET_CHECK = 9
STATE_FIND_EXIT = 10
STATE_ARROW_ALIGN = 11
STATE_ARROW_ALIGN_SIDE_STEP = 12
STATE_VICTORY = 13
STATE_EXIT_ALIGN_PARALLEL = 14
STATE_EXIT_REALIGN_LINE = 15
STATE_EXIT_FOLLOW_LINE = 16

# ------------------------------------------
# 3. Vision General
# ------------------------------------------
X_DEADZONE = 0.05           # Align tolerance (+/-)
Y_DEADZONE = 0.05           # Distance tolerance (+/-)

# ------------------------------------------
# 4. Task: Line Following & Plate
# ------------------------------------------
# Driving
LINE_FOLLOW_EFFORT = -0.22   # Base driving effort
TURN_EFFORT_ADJUSTMENT = -0.15 # Differential effort for turning
LINE_THRESHOLD = 0.9         # Black Line detection threshold

# Plate Interaction
PLATE_APPROACH_EFFORT = -0.20 # Slower effort for precise approach
PLATE_RETURN_SPEED = 20    # Effort to return
PLATE_PARK_DIST = 30.0       # Distance to drive blind onto/off plate (cm) - KEEP DISTANCE

# Plate Vision
X_TARGET_PLATE = 0.5
Y_TARGET_PLATE = 0.85
PLATE_SCORE_THRESHOLD = 0.60
PLATE_CHECK_DURATION = 0.1

# ------------------------------------------
# 5. Task: Basket (Exploration & Pickup)
# ------------------------------------------
# Exploration
EXPLORE_SPEED = -25.0     # Base effort
EXPLORE_TURNING_SPEED = 15.0 # cm/s turning speed
WALL_THRESHOLD = 25.0       # cm (Bounce trigger)
EXPLORE_SPIN_INTERVAL = 50.0 # cm distance to drive before spinning

# Basket Vision
X_TARGET_BASKET = 0.60
Y_TARGET_BASKET = 0.82
BASKET_SCORE_THRESHOLD = 0.92

# Basket Interaction
BASKET_STOP_DIST = 10.0     # cm (Stop before pickup)
PICKUP_DRIVE_DIST = 13.5    # cm
SERVO_PICKUP = 10.0
SERVO_CARRY = 40.0

# ------------------------------------------
# 6. Task: Exit Strategy
# ------------------------------------------
EXIT_APPROACH_EFFORT = -0.3 # Effort
EXIT_SCORE_THRESHOLD = 0.9

# Exit Vision
X_TARGET_ARROW = 0.5
Y_TARGET_ARROW = 0.2
ARROW_AVOID_Y = 0.2         # Wrong-side avoidance threshold 

# ==========================================
# GLOBALS
# ==========================================

# Initialize UART for communication with Coral/External devices
uart = UART(0, baudrate=UART_BAUDRATE, tx=Pin(0), rx=Pin(1), timeout=UART_TIMEOUT)

# ==========================================
# FUNCTIONS
# ==========================================

def get_bboxes():
    """
    Get object detection bounding boxes from UART.
    Decodes the JSON line from the UART buffer.
    """
    line = uart.readline()
    if line is None:
        return None

    try:
        data = json.loads(line.decode("utf-8"))
        return data.get("bboxes", [])
    except Exception:
        return None

def get_target(bboxes, target_id):
    """
    Finds the best bounding box for a specific ID.
    Returns: (x_center, y_center, score) or None
    """
    if not bboxes:
        return None
        
    best_bbox = None
    for bbox in bboxes:
        if bbox["id"] == target_id:
            if best_bbox is None:
                best_bbox = bbox
            else:
                if bbox["score"] > best_bbox["score"]:
                    best_bbox = bbox
                    
    if best_bbox:
        # Calculate center
        width = best_bbox["xmax"] - best_bbox["xmin"]
        height = best_bbox["ymax"] - best_bbox["ymin"]
        x_center = best_bbox["xmin"] + (width / 2.0)
        y_center = best_bbox["ymin"] + (height / 2.0)
        return x_center, y_center, best_bbox["score"]
        
    return None

def clear_uart(duration=0.5):
    """
    Clears the UART buffer for a specified duration.
    Useful to flush old data before starting a critical check.
    """
    start_time = time.time()
    while (time.time() - start_time) < duration:
        get_bboxes()
        time.sleep(0.01)

def follow_line_step():
    """
    Executes one step of line following.
    SENSORS: Mounted on the BACK.
    MOVEMENT: Robot drives BACKWARDS (Negative Speed).
    GOAL: Keep line centered between sensors.
    """
    # 1. Read Sensors
    left_val = reflectance.get_left()
    right_val = reflectance.get_right()
    
    # 2. Simple If/Else Logic
    # We compare the two sensors. If one is significantly darker than the other, we turn towards it.
    # Since we are driving BACKWARD:
    # - To turn the BACK to the LEFT: Left Wheel speed must be more negative (faster back), Right slower.
    # - To turn the BACK to the RIGHT: Right Wheel speed must be more negative (faster back), Left slower.
    
    diff = left_val - right_val
    threshold = 0.1 # Small deadzone to avoid jitter on straight lines

    if diff > threshold:
        # Left sensor sees more line -> Turn Left (Backwards)
        left_effort = LINE_FOLLOW_EFFORT - TURN_EFFORT_ADJUSTMENT
        right_effort = LINE_FOLLOW_EFFORT + TURN_EFFORT_ADJUSTMENT
        drivetrain.set_effort(left_effort, right_effort)
        
    elif diff < -threshold:
        # Right sensor sees more line -> Turn Right (Backwards)
        left_effort = LINE_FOLLOW_EFFORT + TURN_EFFORT_ADJUSTMENT
        right_effort = LINE_FOLLOW_EFFORT - TURN_EFFORT_ADJUSTMENT
        drivetrain.set_effort(left_effort, right_effort)
        
    else:
        # Roughly centered -> Go Straight
        drivetrain.set_effort(LINE_FOLLOW_EFFORT,LINE_FOLLOW_EFFORT)

def wobble():
    imu.reset_yaw()
    clear_uart(1)
    wobble_angle = 45
    found = False
    
    while imu.get_yaw()-wobble_angle < 0:
        time.sleep(0.01)
        current = imu.get_yaw()
        diff = wobble_angle - current
        if diff > 0:
            drivetrain.set_speed(-15,15)
        else:
            drivetrain.set_speed(15,-15)
        bboxes = get_bboxes()
        target = get_target(bboxes, ID_PLATE)
        if target:
            x , y , score = target
            if score > PLATE_SCORE_THRESHOLD:
                found = True
                break
    drivetrain.stop()
    if found:
        return True
    
    while imu.get_yaw()+wobble_angle > 0:
        time.sleep(0.01)
        current = imu.get_yaw()
        diff = -wobble_angle - current
        if diff > 0:
            drivetrain.set_speed(-15,15)
        else:
            drivetrain.set_speed(15,-15)
        bboxes = get_bboxes()
        target = get_target(bboxes, ID_PLATE)
        if target:
            x , y , score = target
            if score > PLATE_SCORE_THRESHOLD:
                found = True
                break
    drivetrain.stop()
    if found:
        return True
    
    while abs(imu.get_yaw()) > 7:
        time.sleep(0.01)
        current = imu.get_yaw()
        print(imu.get_yaw())
        diff = 0 - current
        if diff > 0:
            drivetrain.set_speed(-15,15)
        else:
            drivetrain.set_speed(15,-15)
        bboxes = get_bboxes()
        target = get_target(bboxes, ID_PLATE)
        if target:
            x , y , score = target
            if score > PLATE_SCORE_THRESHOLD:
                found = True
                break
    drivetrain.stop()
    if found:
        clear_uart()
        return True
    clear_uart()
    return False

def turn_to_heading(target_heading):
    """Turns robot to absolute heading using simple difference."""
    current = imu.get_heading()
    diff = target_heading - current
    drivetrain.turn(diff,0.3)

def turn_to_yaw(target_yaw):
    """Turns robot to absolute yaw using simple difference."""
    while abs(imu.get_yaw()-target_yaw) > 2:
        current = imu.get_yaw()
        diff = target_yaw - current
        if diff > 0:
            drivetrain.set_speed(-15,15)
        else:
            drivetrain.set_speed(15,-15)

def check_avoid_arrow(bboxes):
    """Checks for Arrow at top of image. If found, Turns 180 and returns True."""
    target = get_target(bboxes, ID_ARROW)
    if target:
        x, y, score = target
        # If Y is small (Top), arrow is far/high. Wrong side.
        if y < ARROW_AVOID_Y:
            print("Arrow detected too high! Turning 180.")
            drivetrain.turn(180,0.4)
            return True
    return False

def check_avoid_line():
    """Checks for Black Line (Entrance). If found, Turns 180 and returns True."""
    if reflectance.get_left() > LINE_THRESHOLD or reflectance.get_right() > LINE_THRESHOLD:
        print("Line detected (Entrance)! Turning 180 safety.")
        drivetrain.stop()
        drivetrain.straight(10.0) # Back away from line
        drivetrain.turn(180,0.4)
        return True
    return False

# ==========================================
# MAIN
# ==========================================

def main():
    print("XRP Robot - Backward Line Follower - Started")
    
    # we first initialize the XRP
    
    servo_one.set_angle(SERVO_HOME)
    drivetrain.stop()
    imu.calibrate()
    imu.reset_yaw()
    
    global current_state
    current_state = STATE_FIND_LINE

    global dist_last_spin
    dist_last_spin = None

    # Wait for Coral Micro to start running
    print("Waiting for Coral Micro to boot...")
    time.sleep(RESTART_WAIT)
    print("Go!")

    global start_time
    start_time = time.time()
    
    
    try:
        while True:
            # 1. Get Vision Data
            bboxes = get_bboxes()
            
            if current_state == STATE_FIND_LINE:
                # Check for Plate to switch state
                # target = get_target(bboxes, ID_PLATE)
                # if target:
                #     x, y, score = target
                #     # Double check initial threshold
                #     if score >= PLATE_SCORE_THRESHOLD:
                #         print(f"Plate Detected! Score: {score:.2f}. Verifying...")
                #         drivetrain.stop()
                #         imu.reset_yaw()
                #         clear_uart()
                        
                #         # Verification Loop
                #         # verified = True
                #         # start_verify = time.time()
                #         # time.sleep(0.2)
                        
                #         # while (time.time() - start_verify) < PLATE_CHECK_DURATION:
                #         #     bboxes = get_bboxes() # Refresh data
                #         #     check_target = get_target(bboxes, ID_PLATE)
                            
                #         #     if not check_target:
                #         #         print("msg: Verification Failed (Lost).")
                #         #         verified = False
                #         #         break
                            
                #         #     # Check Score
                #         #     if check_target[2] < PLATE_SCORE_THRESHOLD:
                #         #         print(f"msg: Verification Failed (Score {check_target[2]:.2f} < {PLATE_SCORE_THRESHOLD}).")
                #         #         verified = False
                #         #         break
                            
                #         #     time.sleep(0.1)
                            
                #         # if not verified:
                #         #     print("Verification Failed. Resuming Line Follow...")
                #         #     clear_uart()
                #         #     continue

                #         # Success -> Reset & Approach
                #         print("Plate Verified! Resetting Yaw & Approach.")
                #         drivetrain.reset_encoder_position()
                        
                #         current_state = STATE_APPROACH_PLATE
                #         time.sleep(0.1) 
                #         continue

                # Normal Line Following
                follow_line_step()

                if time.time() - start_time > 2:
                    drivetrain.stop()
                    if wobble():
                        drivetrain.stop()
                        current_state = STATE_APPROACH_PLATE
                        continue
                    start_time = time.time()
                    continue
            
            elif current_state == STATE_APPROACH_PLATE:
                # Visual Approach (Backward)
                target = get_target(bboxes, ID_PLATE)
                
                # If lost, keep driving slowly straight back to find it
                if not target:
                    drivetrain.set_effort(PLATE_APPROACH_EFFORT, PLATE_APPROACH_EFFORT)
                    continue
                    
                x, y, score = target
                
                # Check Arrival (BOTH X and Y must be good)
                x_aligned = abs(x - X_TARGET_PLATE) < X_DEADZONE
                y_arrived = y > (Y_TARGET_PLATE - Y_DEADZONE)
                
                if x_aligned and y_arrived:
                    print(f"Visual Approach Done (x:{x:.2f}, y:{y:.2f}). Parking...")
                    drivetrain.stop()
                    
                    # No longer recording distance as per user request
                    
                    current_state = STATE_PARK_PLATE
                    continue
                
                # If we are at the correct distance (Y) but not aligned (X)
                # Pivot in place to fix X
                if y_arrived and not x_aligned:
                    print(f"Y Arrived, Fixing X ({x:.2f})...")
                    drivetrain.stop()
                    
                    # Determine Turn Direction
                    # If x < Target (Left), Turn Tail Left (+Angle)
                    # If x > Target (Right), Turn Tail Right (-Angle)
                    # Note: Positive Turn = Left in this codebase
                    pivot_angle = 5 if x < X_TARGET_PLATE else -5
                    drivetrain.turn(pivot_angle,0.3)
                    time.sleep(0.1) # Stabilization
                    continue

                # Fixed Arcade Control (Bang-Bang) for Approach
                throttle = PLATE_APPROACH_EFFORT
                turn = 0.0
                
                # Turn Logic (Fixed Values)
                # Tail Left (Negative Turn) if Object is Left (x < 0.5)
                # Tail Right (Positive Turn) if Object is Right (x > 0.5)
                if x < (X_TARGET_PLATE - X_DEADZONE):
                    turn = -0.10 # Turn Tail Left
                elif x > (X_TARGET_PLATE + X_DEADZONE):
                    turn = 0.10  # Turn Tail Right
                    
                # Apply Arcade Mix
                l_effort = throttle + turn
                r_effort = throttle - turn
                
                drivetrain.set_effort(l_effort, r_effort)

            elif current_state == STATE_PARK_PLATE:
                print("Parking on Plate (Blind)...")
                # 1. Drive onto Plate
                drivetrain.straight(-PLATE_PARK_DIST) # Backward
                
                # 2. Wait 3 seconds (draining UART)
                print("Waiting...")
                start_wait = time.time()
                while (time.time() - start_wait) < 3.0:
                    get_bboxes() # Drain UART
                    time.sleep(0.1)
                
                # 3. Drive Off Plate
                print("Leaving Plate...")
                drivetrain.straight(PLATE_PARK_DIST) # Forward (Back to Arrival Point)
                
                current_state = STATE_RETURN_TO_LINE
                
            elif current_state == STATE_RETURN_TO_LINE:
                print("Returning to Line (Sensor Only)...")

                current_yaw = imu.get_yaw()
                if current_yaw > 0:
                    turn_to_yaw(30)
                else:
                    turn_to_yaw(-30)
                
                drivetrain.set_speed(PLATE_RETURN_SPEED, PLATE_RETURN_SPEED) # Forward
                
                start_return = time.time()
                while True:
                    # Drain UART
                    get_bboxes()
                    
                    # 1. Check Sensors (Primary Exit)
                    if reflectance.get_left() > LINE_THRESHOLD or reflectance.get_right() > LINE_THRESHOLD:
                        print("Line Found during return!")
                        drivetrain.stop()
                        current_state = STATE_REALIGN_LINE
                        break
                        
                    # 2. Safety Timeout/Max Distance
                    # Just in case we miss the line or start too far
                    avg_dist = (drivetrain.get_left_encoder_position() + drivetrain.get_right_encoder_position()) / 2.0
                    if avg_dist > 200.0: # 2 meters max
                        print("Safety Distance Reached (200cm). Stopping.")
                        break
                        
                    time.sleep(0.01)
                    
                drivetrain.stop()
            
            elif current_state == STATE_REALIGN_LINE:
                if reflectance.get_left() > LINE_THRESHOLD:
                    drivetrain.set_speed(10,-10)
                    while reflectance.get_right() < LINE_THRESHOLD:
                        drivetrain.straight(2,0.4)
                        time.sleep(0.01)
                    drivetrain.stop()
                    current_state = STATE_FOLLOW_TO_END
                elif reflectance.get_right() > LINE_THRESHOLD:
                    drivetrain.set_speed(-10,10)
                    while reflectance.get_left() < LINE_THRESHOLD:
                        drivetrain.straight(2,0.4)
                        time.sleep(0.01)
                    drivetrain.stop()
                    current_state = STATE_FOLLOW_TO_END
                else:
                    drivetrain.set_speed(-10,-10)
                    
            elif current_state == STATE_FOLLOW_TO_END:
                # 1. Check Intersection (Stop Condition)
                # Both sensors seeing Black

                 # Wall
                dist = rangefinder.distance()
                if dist < WALL_THRESHOLD and dist > 0:
                    print("Wall Reached (Final)!")
                    drivetrain.stop()
                    current_state = STATE_EXPLORE
                    continue

                if reflectance.get_left() > LINE_THRESHOLD and reflectance.get_right() > LINE_THRESHOLD:
                    print("End of Line / Intersection Reached!")
                    drivetrain.stop()
                    
                    # CAPTURE ENTRANCE HEADING
                    print("Entrance Reached! Resetting Heading to 0.")
                    imu.reset_yaw()
                    
                    # Go forward (continue backward) 20 cm
                    print("Driving forward 20cm...")
                    drivetrain.straight(-20.0)
                    
                    # Next State: Basket Search
                    time.sleep(1.0) 
                    
                    current_state = STATE_EXPLORE
                    drivetrain.reset_encoder_position()
                    continue
                    
                # 2. Normal Line Following
                follow_line_step()

            # elif current_state == STATE_EXPLORE_DUMB:
            #     # 1. Check Line (Entrance Safety) - Priority!
            #     if check_avoid_line():
            #         drivetrain.set_speed(EXPLORE_SPEED, EXPLORE_SPEED)
            #         continue

            #     # 2. Check Arrow Avoidance (If Exit seen from wrong side)
            #     """ if check_avoid_arrow(bboxes):
            #         drivetrain.set_speed(EXPLORE_SPEED, EXPLORE_SPEED)
            #         continue """

            #     # 3. Check Basket
            #     target = get_target(bboxes, ID_BASKET)
            #     if target:
            #         print("Basket Found! Switching to Fine Tune.")
            #         drivetrain.stop()
            #         current_state = STATE_FINE_TUNE_BASKET
            #         continue
                    
            #     # 4. Explore Logic (Wall Follow / Random)
            #     # Simple Wall Bounce
            #     dist = rangefinder.distance()
            #     if dist < WALL_THRESHOLD and dist > 0:
            #         print("Wall Detected (Explore). Turning Left 90.")
            #         drivetrain.stop()
            #         drivetrain.turn(90,0.2) # Turn Left
            #         drivetrain.set_speed(-20,-20)
            #         continue
                
            #     drivetrain.set_speed(EXPLORE_SPEED, EXPLORE_SPEED)

            elif current_state == STATE_EXPLORE:
                
                # ----------------------------------
                # 1. Check Line (Entrance Safety) - PRIORITY
                # ----------------------------------
                if check_avoid_line():
                    # We turned 180 degrees in the helper function.
                    # Reset the distance tracker to the current average to avoid 
                    # triggering a 360 spin immediately after avoiding the line.
                    drivetrain.reset_encoder_position()
                    dist_last_spin = abs((drivetrain.get_left_encoder_position() + drivetrain.get_right_encoder_position()) / 2)
                    drivetrain.set_speed(EXPLORE_SPEED, EXPLORE_SPEED)
                    continue

                # ----------------------------------
                # 2. Check Arrow (Exit Safety) - SMART TURN
                # ----------------------------------
                # If we see an arrow while exploring, it's the exit. Turn away, 
                # but keep looking for the basket.
                target_arrow = get_target(bboxes, ID_ARROW)
                if target_arrow:
                    print("Arrow seen! Smart turning away (130 deg)...")
                    drivetrain.stop()
                    
                    # Start values for the avoidance turn
                    start_yaw = imu.get_yaw()
                    drivetrain.set_speed(EXPLORE_TURNING_SPEED, -EXPLORE_TURNING_SPEED) # Start turning (Tail Left)
                    
                    turn_done = False
                    
                    while not turn_done:
                        # A. Have we reached 130 degrees?
                        if abs(imu.get_yaw() - start_yaw) >= 130:
                            turn_done = True
                            drivetrain.reset_encoder_position()
                            dist_last_spin = abs((drivetrain.get_left_encoder_position() + drivetrain.get_right_encoder_position()) / 2)
                            break
                        
                        # B. Vision Check (Basket) - Priority!
                        # Get fresh data.
                        bboxes = get_bboxes()
                        if get_target(bboxes, ID_BASKET):
                            print("Basket detected during avoidance turn! Switching state.")
                            current_state = STATE_FINE_TUNE_BASKET
                            turn_done = True
                            break # Break the inner turn loop
                        
                        # C. Wall Check (Safety)
                        dist = rangefinder.distance()
                        if dist < WALL_THRESHOLD and dist > 0:
                            print("Wall detected during avoidance turn! Stopping.")
                            # Stop the turn. 
                            # The normal Wall Bounce logic (Point 5) will handle it in the next main loop cycle.
                            turn_done = True
                            break
                        
                        time.sleep(0.01)
                    
                    drivetrain.stop()
                    
                    # IMPORTANT: If we broke out because of the basket, restart main loop to switch state
                    if current_state == STATE_FINE_TUNE_BASKET:
                        continue
                        
                    # Otherwise: Avoidance done, resume driving
                    drivetrain.set_speed(EXPLORE_SPEED, EXPLORE_SPEED)
                    continue

                # ----------------------------------
                # 3. Check Basket (Primary Goal - Instant Check)
                # ----------------------------------
                target = get_target(bboxes, ID_BASKET)
                if target:
                    print("Basket Found! Switching to Fine Tune.")
                    drivetrain.stop()
                    current_state = STATE_FINE_TUNE_BASKET
                    continue
                
                # ----------------------------------
                # 4. Check 360 Spin Condition (Distance)
                # ----------------------------------
                # Calculate average of both encoders for accurate distance despite turning

                if not dist_last_spin:
                    drivetrain.reset_encoder_position()
                    dist_last_spin = abs((drivetrain.get_left_encoder_position() + drivetrain.get_right_encoder_position()) / 2)
                
                current_avg_pos = abs((drivetrain.get_left_encoder_position() + drivetrain.get_right_encoder_position()) / 2)
                
                if (current_avg_pos - dist_last_spin) > EXPLORE_SPIN_INTERVAL:
                    print(f"Distance {EXPLORE_SPIN_INTERVAL}cm reached. Starting Smart 360 Check...")
                    drivetrain.stop()
                    
                    # Start values for spin
                    start_yaw = imu.get_yaw()
                    drivetrain.set_speed(EXPLORE_TURNING_SPEED, -EXPLORE_TURNING_SPEED) # Start turning
                    
                    spin_complete = False
                    
                    while not spin_complete:
                        # A. Reached 360 degrees?
                        if abs(imu.get_yaw() - start_yaw) >= 360:
                            spin_complete = True
                            break
                        
                        # B. Vision Check (Basket)
                        bboxes = get_bboxes() # Always fresh data!
                        if get_target(bboxes, ID_BASKET):
                            print("Basket detected during spin! Switching state.")
                            current_state = STATE_FINE_TUNE_BASKET
                            spin_complete = True
                            break
                        
                        # C. Wall Check (Safety)
                        dist = rangefinder.distance()
                        if dist < WALL_THRESHOLD and dist > 0:
                            print("Wall detected during spin! Aborting.")
                            spin_complete = True
                            break
                        
                        time.sleep(0.01)
                    
                    drivetrain.stop()
                    
                    # Reset distance tracker to current average
                    dist_last_spin = abs((drivetrain.get_left_encoder_position() + drivetrain.get_right_encoder_position()) / 2)
                    
                    # If basket found -> restart main loop
                    if current_state == STATE_FINE_TUNE_BASKET:
                        continue
                    
                    # Otherwise resume
                    drivetrain.set_speed(EXPLORE_SPEED, EXPLORE_SPEED)

                # ----------------------------------
                # 5. Explore Logic (Smart Wall Bounce with Random Turn)
                # ----------------------------------
                dist = rangefinder.distance()
                if dist < WALL_THRESHOLD and dist > 0:
                    print("Wall Detected! Calculating Random Smart Turn...")
                    drivetrain.stop()
                    drivetrain.straight(10.0)
                    clear_uart()
                    
                    # Logic: We want to avoid the sector +/- 45 degrees in front of us (the wall).
                    # The remaining "safe" sector is 270 degrees.
                    # We pick a random angle between 45 and 315 degrees.
                    turn_target = random.randint(45, 135)
                    print(f"Target Turn Angle: {turn_target} deg")
                    
                    start_yaw = imu.get_yaw()
                    # Turn with fixed speed (Tail Left)
                    drivetrain.set_speed(-EXPLORE_TURNING_SPEED, +EXPLORE_TURNING_SPEED) 
                    
                    turn_done = False
                    
                    while not turn_done:
                        # A. Have we reached the target angle?
                        if abs(imu.get_yaw() - start_yaw) >= turn_target:
                            turn_done = True
                            drivetrain.reset_encoder_position()
                            dist_last_spin = abs((drivetrain.get_left_encoder_position() + drivetrain.get_right_encoder_position()) / 2)
                            break
                        
                        # B. Vision Check (Basket) - IMPORTANT!
                        bboxes = get_bboxes()
                        if get_target(bboxes, ID_BASKET):
                            print("Basket detected during wall bounce! Switching State.")
                            current_state = STATE_FINE_TUNE_BASKET
                            turn_done = True
                            break
                        
                        # Note: We do NOT check for walls here, as we are spinning in place
                        # specifically to get away from the wall.
                        
                        time.sleep(0.01)
                    
                    drivetrain.stop()
                    
                    # If broken due to basket -> Restart loop for state switch
                    if current_state == STATE_FINE_TUNE_BASKET:
                        continue
                        
                    # Otherwise: Turn done. Resume driving gently (Backwards).
                    print("Turn done. Resuming explore...")
                    drivetrain.set_speed(EXPLORE_SPEED, EXPLORE_SPEED)
                    continue
                
                # ----------------------------------
                # 6. Default Drive
                # ----------------------------------
                drivetrain.straight(-5,0.4)
                clear_uart(1)
                print("Exploring...")

            elif current_state == STATE_FINE_TUNE_BASKET:
                print("Fine Tuning Basket...")
                
                target = get_target(bboxes, ID_BASKET)

                if not target:
                    print("Basket Lost! Exploring...")
                    current_state = STATE_EXPLORE
                    continue

                x, y, score = target
                y_error_sum = 0
                x_error_sum = 0

                basket_there = True

                Kp_x = 12
                Ki_x = 2
                Kp_y = 25
                Ki_y = 6
                err_tresh_x = 0.015
                err_tresh_y = 0.015
                err_not_zero = 0

                while err_not_zero < 1 and basket_there:
                    bboxes = get_bboxes()
                    target = get_target(bboxes, ID_BASKET)
                    if not target:
                        print("Basket Lost!")
                        basket_there = False
                        # security to avoid going into the wall
                        # if y > Y_TARGET_BASKET-0.05:
                        #      drivetrain.straight(10.0)
                        drivetrain.stop()
                        clear_uart()
                        continue

                    x, y, score = target

                    y_error = y - Y_TARGET_BASKET
                    x_error = x - X_TARGET_BASKET

                    y_error_sum += y_error
                    x_error_sum += x_error

                    if abs(x_error) < err_tresh_x:
                        x_error_sum = 0
                    if abs(y_error) < err_tresh_y:
                        y_error_sum = 0

                    if abs(x_error) < err_tresh_x and abs(y_error) < err_tresh_y:
                        err_not_zero += 1
                        drivetrain.stop()
                        continue
                    else:
                        err_not_zero = 0    

                    turn_x = Kp_x * x_error + Ki_x * x_error_sum
                    go_y = Kp_y * y_error + Ki_y * y_error_sum

                    left_speed = max(min(go_y + turn_x, 20), -20)
                    right_speed = max(min(go_y - turn_x, 20), -20)
                    drivetrain.set_speed(left_speed, right_speed)
                    time.sleep(0.01)
                
                drivetrain.stop()

                # we check the basket again while still

                if basket_there:
                    # clear_uart()
                    # bboxes = get_bboxes()
                    # target = get_target(bboxes, ID_BASKET)
                    # if not target:
                    #     print("Basket Lost!")
                    #     basket_there = False
                    #     drivetrain.straight(5.0)
                    #     drivetrain.stop()
                    #     clear_uart()
                    #     continue

                    # x, y, score = target
                    # x_error = x - X_TARGET_BASKET
                    # y_error = y - Y_TARGET_BASKET
                    
                    # if abs(x_error) < err_tresh_x and abs(y_error) < err_tresh_y:
                    #     basket_there = False
                    #     drivetrain.straight(5.0)
                    #     drivetrain.stop()
                    #     clear_uart()
                    #     continue

                    if score >= BASKET_SCORE_THRESHOLD:
                        current_state = STATE_PICKUP_BASKET
                    elif score < BASKET_SCORE_THRESHOLD:
                        drivetrain.straight(10.0)
                        clear_uart()
               
            elif current_state == STATE_PICKUP_BASKET:
                print("Picking up Basket...")
                
                # 1. Lower Arm
                servo_one.set_angle(SERVO_PICKUP)
                time.sleep(1.0)
                
                # 2. Drive Backward (Scoop)
                drivetrain.straight(-PICKUP_DRIVE_DIST,0.4) 
                
                # 3. Raise Arm
                # Lift slowly?
                for angle in range(int(SERVO_PICKUP), int(SERVO_CARRY), 5):
                    servo_one.set_angle(angle)
                    time.sleep(0.1)
                servo_one.set_angle(SERVO_CARRY)
                
                print("Basket Secured.")
                
                # 4. SAFETY: Drive Forward away from wall/corner
                print("Backing away from wall...")
                drivetrain.straight(20.0) # Forward (Away)
                
                print("Finding Exit...")
                drivetrain.turn(-30,0.4)
                current_state = STATE_BASKET_CHECK
                clear_uart(3)

            elif current_state == STATE_BASKET_CHECK:
                print("Checking Basket...")
                
                target = get_target(bboxes, ID_BASKET)
                safety_count = 0
                while not target and safety_count < 10:
                    bboxes = get_bboxes()
                    target = get_target(bboxes, ID_BASKET)
                    safety_count += 1

                if not target:
                    print("Basket Lost for too long, exploring again!")
                    drivetrain.stop()
                    servo_one.set_angle(SERVO_PICKUP)
                    drivetrain.straight(5.0)
                    servo_one.set_angle(SERVO_HOME)
                    clear_uart(2)
                    current_state = STATE_FINE_TUNE_BASKET
                    continue
                
                x, y, score = target
                
                if x < X_TARGET_BASKET - 0.1:
                    current_state = STATE_FINE_TUNE_BASKET
                    print("Not picked up Basket... trying again")
                    servo_one.set_angle(SERVO_HOME)
                    continue
                else:
                    current_state = STATE_FIND_EXIT
                    print("Picked up Basket... finding exit")
                    clear_uart()
                    drivetrain.set_speed(EXPLORE_SPEED, EXPLORE_SPEED)
            
            elif current_state == STATE_FIND_EXIT:
                # Goal: Find Arrow and drive under it
                print("Finding Exit...")

                target = get_target(bboxes, ID_ARROW)

                if target:
                    print("arrow found!")
                    drivetrain.stop()
                    x,y,score = target
                    if score >= 0.7:
                        current_state = STATE_ARROW_ALIGN
                        continue
                
                # 1. Check Line (Entrance Safety)
                if check_avoid_line():
                    drivetrain.set_speed(EXPLORE_SPEED, EXPLORE_SPEED)
                    continue
                               
                # 2. Exploration (If no arrow)
                dist = rangefinder.distance()
                if dist < WALL_THRESHOLD and dist > 0:
                    print("Wall (Exit Search). Turning.")
                    drivetrain.stop()
                    drivetrain.turn(90,0.4)
                    drivetrain.set_speed(EXPLORE_SPEED, EXPLORE_SPEED)
                    continue
                
            elif current_state == STATE_ARROW_ALIGN:
                target = get_target(bboxes, ID_ARROW)

                if dist < WALL_THRESHOLD and dist > 0:
                    print("Wall (Arrow Alignment). Turning.")
                    drivetrain.stop()
                    drivetrain.straight(10.0)
                    drivetrain.turn(90,0.4)
                    drivetrain.set_speed(EXPLORE_SPEED, EXPLORE_SPEED)
                    current_state = STATE_FIND_EXIT
                    continue


                safety_count = 0
                # checks for the arrow 10 times, if it finds it once it's ok
                while not target and safety_count < 10:
                    bboxes = get_bboxes()
                    target = get_target(bboxes, ID_ARROW)
                    safety_count += 1

                if not target:
                    print("Arrow Lost! Exploring...")
                    current_state = STATE_FIND_EXIT
                    drivetrain.stop()
                    clear_uart()
                    drivetrain.set_speed(EXPLORE_SPEED, EXPLORE_SPEED)
                    continue

                x, y, score = target
                y_error_sum = 0
                x_error_sum = 0

                arrow_not_there = 0
                err_not_zero = 0

                Kp_x = 12
                Ki_x = 3
                Kp_y = 20
                Ki_y = 6
                err_tresh_x = 0.04
                err_tresh_y = 0.05
                
                while err_not_zero < 1 and arrow_not_there < 10:
                    bboxes = get_bboxes()
                    target = get_target(bboxes, ID_ARROW)
                    if not target:
                        print("Arrow Lost!")
                        arrow_not_there +=1
                        # security to avoid going into the wall
                        # if y > Y_TARGET_BASKET-0.05:
                        #      drivetrain.straight(10.0)
                        drivetrain.stop()
                        continue
                    else:
                        arrow_not_there = 0

                    x, y, score = target

                    y_error = - y + Y_TARGET_ARROW
                    x_error = x - X_TARGET_ARROW

                    y_error_sum += y_error
                    x_error_sum += x_error

                    # stopping integral windup
                    if abs(x_error) < err_tresh_x:
                        x_error_sum = 0
                    if abs(y_error) < err_tresh_y:
                        y_error_sum = 0

                    if abs(x_error) < err_tresh_x and abs(y_error) < err_tresh_y:
                        err_not_zero += 1
                        drivetrain.stop()
                        continue
                    else:
                        err_not_zero = 0    

                    turn_x = Kp_x * x_error + Ki_x * x_error_sum
                    go_y = Kp_y * y_error + Ki_y * y_error_sum

                    left_speed = max(min(go_y + turn_x, 20), -20)
                    right_speed = max(min(go_y - turn_x, 20), -20)
                    drivetrain.set_speed(left_speed, right_speed)
                    time.sleep(0.01)
                
                drivetrain.stop()

                if arrow_not_there < 1:
                    current_state = STATE_ARROW_ALIGN_SIDE_STEP
                else:
                    current_state = STATE_FIND_EXIT
                    drivetrain.stop()
                    clear_uart()
                    drivetrain.set_speed(EXPLORE_SPEED, EXPLORE_SPEED)
                
            elif current_state == STATE_ARROW_ALIGN_SIDE_STEP:
                heading = imu.get_heading()
                diff_heading = heading - 180
                if abs(diff_heading) > 5.0:
                    side_dist = diff_heading * 0.5
                    turn_to_heading(90)
                    drivetrain.straight(side_dist)
                    drivetrain.stop()
                    turn_to_heading(180)
                    current_state = STATE_ARROW_ALIGN
                    clear_uart(1)
                    continue
                else:
                    #Go out blind
                    drivetrain.straight(-100)
                    drivetrain.stop()
                    current_state = STATE_EXIT_ALIGN_PARALLEL
                    continue

            # elif current_state == STATE_ARROW_ALIGN_BAD:
            #     # We check for x alignment

            #     if y < Y_TARGET_ARROW and score > EXIT_SCORE_THRESHOLD:
            #         # Check Heading & Alignment (Perpendicularity)
                    
            #         target_heading = 180.0 # Reset at entrance means Exit is 180
                    
            #         # 1. Fix Heading First
            #         if abs(heading_error) > 5.0:
            #             print(f"Close but Bad Heading (Err:{heading_error:.1f}). Turning to fix...")
            #             drivetrain.stop()
            #             turn_to_heading(target_heading)
            #             time.sleep(0.2)
            #             continue
                        
            #         # 2. Fix Lateral Offset (Side Step)
            #         # If x is not centered, we are parallel but shifted.
            #         # Needs 90 deg turn, drive, back 90.
            #         if abs(x - X_TARGET_ARROW) > 0.1:
            #             print(f"Close, Parallel, but Shifted X ({x:.2f}). Side-Stepping...")
            #             drivetrain.stop()
                        
            #             # Move Lateral
            #             # Turn 90 Left
            #             drivetrain.turn(90)
                        
            #             # Drive Proportional to Error
            #             # If x > 0.5 (Right), we need to move Right.
            #             # Rear-Right is Negative Drive (Backwards).
            #             correction = (x - X_TARGET_ARROW) * 30.0 # Scale
            #             correction = max(min(correction, 15.0), -15.0) # Limit Step
                        
            #             drivetrain.straight(-correction) 
                        
            #             # Turn Back -90
            #             turn_to_heading(target_heading)
            #             continue

            #         print("Under Arrow, Parallel & Centered! Exiting...")
            #         drivetrain.straight(-50.0) # Blind Exit
            #         current_state = STATE_VICTORY
            #         continue
                    
            #     # Visual Approach
            #     Kp_x = 10
            #     Ki_x = 3
            #     x_error = x - X_TARGET_ARROW
            #     x_error_sum += x_error
            #     turn_correction = x_error * Kp_x + x_error_sum * Ki_x
                
            #     l_speed = turn_correction
            #     r_speed = -turn_correction
            #     drivetrain.set_speed(l_speed, r_speed)

            elif current_state == STATE_EXIT_ALIGN_PARALLEL:
                print("Aligning Parallel to Exit (90 deg)...")
                drivetrain.stop()
                turn_to_heading(90)
                
                print("Searching for line (Forward)...")
                # Drive one way
                found_line = False
                drive_speed = 20
                
                # Check rapidly while driving
                # We want to drive ~40cm? 
                start_search = time.time()
                drivetrain.set_speed(drive_speed, drive_speed)
                
                while (time.time() - start_search) < 2.0: # ~2 seconds
                    if reflectance.get_left() > LINE_THRESHOLD or reflectance.get_right() > LINE_THRESHOLD:
                         found_line = True
                         drivetrain.stop()
                         break
                    time.sleep(0.01)
                
                if found_line:
                    drivetrain.stop()
                    print("Line Found!")
                    current_state = STATE_EXIT_REALIGN_LINE
                    continue
                    
                print("Not found. Searching other way (Double dist)...")
                drivetrain.stop()
                drivetrain.set_speed(-drive_speed, -drive_speed)
                start_search = time.time()
                
                while (time.time() - start_search) < 4.0: # ~4 seconds
                    if reflectance.get_left() > LINE_THRESHOLD or reflectance.get_right() > LINE_THRESHOLD:
                         found_line = True
                         drivetrain.stop()
                         break
                    time.sleep(0.01)
                    
                drivetrain.stop()
                if found_line:
                    print("Line Found!")
                    current_state = STATE_EXIT_REALIGN_LINE
                else:
                    print("Line NEVER Found! Giving up -> Victory.")
                    current_state = STATE_VICTORY
                    
            elif current_state == STATE_EXIT_REALIGN_LINE:
                print("Realigning to Line...")
                # 1. Turn to intercept angle (45)
                turn_to_heading(135)
                
                # 2. Re-use simple alignment logic (Pivot until other sensor sees line)
                # Assuming we are ON the line (or crossed it). 
                # If we just crossed it, we might need to back up or just pivot.
                
                # Let's try the logic: user said "heading 45 e poi return to align"
                # return to align logic (from STATE_REALIGN_LINE):
                
                if reflectance.get_left() > LINE_THRESHOLD:
                    drivetrain.set_speed(10,-10)
                    while reflectance.get_right() < LINE_THRESHOLD:
                        time.sleep(0.01)
                    drivetrain.stop()
                elif reflectance.get_right() > LINE_THRESHOLD:
                    drivetrain.set_speed(-10,10)
                    while reflectance.get_left() < LINE_THRESHOLD:
                        time.sleep(0.01)
                    drivetrain.stop()
                else:
                    current_state = STATE_EXIT_ALIGN_PARALLEL
                    continue
                    
                current_state = STATE_EXIT_FOLLOW_LINE
                
            elif current_state == STATE_EXIT_FOLLOW_LINE:
                # 1. Check Stop Conditions (Wall OR Cross Line)
                
                # Wall
                dist = rangefinder.distance()
                if dist < WALL_THRESHOLD and dist > 0:
                    print("Wall Reached (Final)!")
                    drivetrain.stop()
                    current_state = STATE_VICTORY
                    continue
                    
                # Cross Line (Both Black)
                if reflectance.get_left() > LINE_THRESHOLD and reflectance.get_right() > LINE_THRESHOLD:
                     print("End of Line Reached (Final)!")
                     drivetrain.stop()
                     current_state = STATE_VICTORY
                     continue
                     
                # 2. Follow Line
                follow_line_step()

            elif current_state == STATE_VICTORY:
                print("Exited! Dropping Basket...")
                drivetrain.stop()
                
                # 1. Drop Basket
                servo_one.set_angle(SERVO_PICKUP)
                time.sleep(1.0)
                
                # 2. Back Away (Drive Forward, away from rear-mounted basket)
                print("Moving away...")
                drivetrain.straight(20.0)
                
                # 3. Raise Arm (optional, to look cool/safe)
                servo_one.set_angle(SERVO_CARRY)
                
                # 4. Victory Spin
                print("Victory Spin!")
                drivetrain.turn(360)
                
                drivetrain.stop()
                print("Mission Complete.")
                break # Exit Loop
            
            # Add small delay for loop stability
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        drivetrain.stop()
        print("Stopped.")

if __name__ == "__main__":
    main()