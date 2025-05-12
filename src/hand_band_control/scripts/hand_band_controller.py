#!/usr/bin/env python3
"""
Controller Node
This node subscribes to EmgArray messages published by myo-rawNode and
controls the hand according to EMG readings.
"""
import rospy
from ros_myo.msg import EmgArray      # EMG data message from ros_myo package
from std_msgs.msg import Float64       # Used for publishing single float commands
import matplotlib.pyplot as plt
import numpy as np
import serial
import time

# Global variable to store the latest received EMG data (8 channels)
latest_emg = None

# Global variable to track the current hand state ("open" or "closed")
hand_state = "open"  

# Grasp and release thresholds for EMG signals
grasp_threshold = 0
release_threshold = 0


# Publishers for each finger controller (will be initialized in main)
thumb_pub = None
index_pub = None
mrl_pub = None

# Arduino serial port settings
arduino_port = "/dev/ttyACM0"  
baud_rate = 9600          
rotation_speed = 800
rotation_accel = 300

def calibrate_emg_thresholds():
    """
    Prompts user for 3 grasp and 3 release actions with on-screen instructions and countdown.
    Collects EMG data and returns adaptive thresholds for grasp and release.
    """
    time.sleep(1.0)
    global grasp_threshold, release_threshold
    global latest_emg
    grasp_max = []
    release_max = []

    print("💪 EMG calibration is starting. Please follow the instructions.")
    rospy.sleep(1.0)

    for i in range(3):
        print(f"\n[{i+1}/3] 👊 Please SQUEEZE your hand. The measurement will take 3 seconds...")
        start_time = time.time()
        peak_grasp = 0
        while time.time() - start_time < 3:
            if latest_emg is not None:
                val = latest_emg[0]
                if val > peak_grasp:
                    peak_grasp = val
            time.sleep(0.01)
        grasp_max.append(peak_grasp)
        print(f"👊 Grasp peak value: {peak_grasp}")

        print("✋ Now RELEASE your hand for 3 seconds.")
        start_time = time.time()
        peak_release = 0
        while time.time() - start_time < 3:
            if latest_emg is not None:
                val = latest_emg[2]
                if val > peak_release:
                    peak_release = val
            time.sleep(0.01)
        release_max.append(peak_release)
        print(f"✋ Release peak value: {peak_release}")

    # Set thresholds
    grasp_threshold = max(min(grasp_max) - 100, 0)
    release_threshold = max(min(release_max) - 100, 0)

    print(f"\n✅ Calibration completed.")
    print(f"    ➤ Grasp threshold: {grasp_threshold}")
    print(f"    ➤ Release threshold: {release_threshold}\n")

    return grasp_threshold, release_threshold


def rotate_wrist(rotation_angle_deg):
    try:
        angle_int = int(rotation_angle_deg)  # Arduino expects integer values
        cmd_str = f"CMD:{rotation_speed},{rotation_accel},{angle_int}\n"
        
        # Serial bağlantıyı aç
        with serial.Serial('/dev/ttyACM0', baud_rate, timeout=2) as ser:
            time.sleep(1)  # wait for Arduino get ready
            ser.reset_input_buffer()  # clean old data
            ser.write(cmd_str.encode())  # send the command
            rospy.loginfo(f"[Arduino] Sent: {cmd_str.strip()}")
            
            # read response
            response = ser.readline()
            try:
                decoded = response.decode('utf-8').strip()
                rospy.loginfo(f"[Arduino] Response: {decoded}")
            except UnicodeDecodeError:
                rospy.logwarn("[Arduino] Received non-text response.")
                rospy.logwarn(f"[Arduino] Raw bytes: {list(response)}")

    except serial.SerialException as e:
        rospy.logwarn(f"[Arduino] Serial communication error: {e}")
    except Exception as e:
        rospy.logwarn(f"[Arduino] Unexpected error: {e}")

def get_index_for_lateral_grasp(grasp_gap_mm):
    """
    Returns the index joint value for lateral grasp.
    Special cases handled for gap = 0 and gap > 55.
    """
    gap_index_map = [
        (0,    0.9),
        (7,    0.8),
        (10,  -0.9),
        (12,  -0.8),
        (15,  -0.7),
        (18,  -0.6),
        (20,  -0.5),
        (25,  -0.4),
        (30,  -0.3),
        (35,  -0.2),
        (40,  -0.1),
        (45,   0.3),
        (50,  -0.1),
        (52,   0.0),
        (55,   0.1)
    ]

    if grasp_gap_mm == 0:
        return 0.9
    elif 0 < grasp_gap_mm <= 55:
        for threshold, index_val in gap_index_map:
            if grasp_gap_mm == 55:
                return gap_index_map[-1][1]
            elif grasp_gap_mm < threshold:
                return index_val
    else:
        # Out of defined range – can be handled as error condition
        return 2


def compute_thumb_index_palmar(grasp_gap_mm):
    """
    Given a desired gap (in mm), returns corresponding thumb and index joint values.
    """

    compensation = 5.0
    grasp_gap_mm += compensation

    # Clamp the input to the practical range based on your measurements
    gap = max(0, min(grasp_gap_mm, 100))

    # Max/min values from your system
    max_gap = 100.0
    min_gap = 0.0

    max_thumb = 0.59
    min_thumb = 0.0

    max_index = 1.0
    min_index = 0.0

    # Define smooth drop pattern: use a non-linear drop off (exponential-like fit)
    # These factors are tuned empirically to fit your table

    # Thumb decreases faster than index with increasing gap
    thumb_value = max_thumb * (1 - (gap / max_gap) ** 1.2)
    index_value = max_index * (1 - (gap / max_gap) ** 1.0)

    # Clamp to avoid overstepping bounds
    thumb_value = max(min_thumb, min(thumb_value, max_thumb))
    index_value = max(min_index, min(index_value, max_index))

    return round(thumb_value, 3), round(index_value, 3)


def emg_callback(msg):
    """
    Subscriber callback function:
    Updates the global latest_emg variable with each new EmgArray message.
    """
    global latest_emg
    latest_emg = msg.data


def process_emg_and_publish(emg_data, grasp_type, grasp_size_mm, rotation_angle_deg, grasp_threshold, release_threshold):
    """
    Processes EMG data to control the hand according to grasp type and size.
    Grasp size affects joint angles if grasp_type is palmar.
    """
    global hand_state, thumb_pub, index_pub, mrl_pub

    # Palmar grasp - open (default position)
    thumb_open_pal = 0.0
    index_open_pal = 0.0
    mrl_open_pal = 0.0

    # Lateral grasp - open
    thumb_open_lat = 0.0
    index_open_lat = 0.9
    mrl_open_lat = 1.06

    thumb = 0.6
    index = -0.9
    mrl = 1.06
    
    # Check if EMG data is available
    if emg_data is None:
        return

    
    # Check for grasp condition (channels 0 and 1):
    if emg_data[0] > grasp_threshold:
        if hand_state != "closed":
            rospy.loginfo(f"{grasp_type.capitalize()} grasp initiated: rotating wrist before closing hand.")
            rotate_wrist(rotation_angle_deg)
            rospy.sleep(1.0)

            if grasp_type == "palmar":
                thumb, index = compute_thumb_index_palmar(grasp_size_mm)
                mrl = index + 0.06
            elif grasp_type == "lateral":
                index = get_index_for_lateral_grasp(grasp_size_mm)
                thumb = 0.69
                mrl = 1.06
                if index == 2:
                    rospy.logwarn("Grasp gap is out of range. Skipping action.")
                    thumb, index, mrl = thumb_open_lat, index_open_lat, mrl_open_lat
            else:
                rospy.logwarn(f"Unknown grasp type: {grasp_type}. Skipping action.")
                return

            thumb_pub.publish(Float64(thumb))
            index_pub.publish(Float64(index))
            mrl_pub.publish(Float64(mrl))
            hand_state = "closed"

    # Check for release condition (channel 2):
    elif emg_data[2] > release_threshold:
        if hand_state != "open":
            rospy.loginfo("Release activated: Opening hand to default position.")

            if grasp_type == "palmar":
                thumb, index, mrl = thumb_open_pal, index_open_pal, mrl_open_pal
            elif grasp_type == "lateral":
                thumb, index, mrl = thumb_open_lat, index_open_lat, mrl_open_lat
            else:
                rospy.logwarn(f"Unknown grasp type: {grasp_type}. Skipping release.")
                return

            thumb_pub.publish(Float64(thumb))
            index_pub.publish(Float64(index))
            mrl_pub.publish(Float64(mrl))
            hand_state = "open"
            rotate_wrist(-rotation_angle_deg)

def main():
    global thumb_pub, index_pub, mrl_pub
    rospy.init_node('hand_band_controller', anonymous=True)
    
    # Subscribe to the EMG topic. Adjust the topic name if needed.
    rospy.Subscriber('/myo_raw/myo_emg', EmgArray, emg_callback, queue_size=1)

    # --- Default control inputs (can later be replaced by another node)
    grasp_type = "palmar"
    grasp_size_mm = 20
    rotation_angle_deg = 50
    
    # Initialize publishers for the individual finger commands.
    thumb_pub = rospy.Publisher("j_thumb_fle_pos_vel_controller/command", Float64, queue_size=1000)
    index_pub = rospy.Publisher("j_index_fle_pos_vel_controller/command", Float64, queue_size=1000)
    mrl_pub   = rospy.Publisher("j_mrl_fle_pos_vel_controller/command", Float64, queue_size=1000)
    
    grasp_threshold, release_threshold = calibrate_emg_thresholds()

    # Optional: set up a live visualization for EMG data using matplotlib.
    plt.ion()
    fig, ax = plt.subplots()
    bars = ax.bar(range(8), np.zeros(8), align='center')
    ax.set_ylim(0, 2048)  # Adjust Y-axis limit based on expected range of EMG data.
    ax.set_xlabel("EMG Channels")
    ax.set_ylabel("Activation")
    ax.set_title("Live EMG Data")
    plt.show()

    rate = rospy.Rate(10)  # Run the loop at 10 Hz.
    while not rospy.is_shutdown():
        if latest_emg is not None:
            process_emg_and_publish(latest_emg, grasp_type, grasp_size_mm, rotation_angle_deg, grasp_threshold, release_threshold)
            for i, bar in enumerate(bars):
                bar.set_height(latest_emg[i])
            fig.canvas.draw()
            fig.canvas.flush_events()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass