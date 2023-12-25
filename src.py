from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from math import sin, cos, radians, atan2, degrees
from dronekit_sitl import SITL
from geopy.distance import geodesic

# Coordinates of points A and B
point_a = (50.4507390, 30.4612420)
point_b = (50.4433260, 30.4480780)

# Flight altitude (in meters)
altitude = 10

# Yaw angle (in degrees)
azimuth_yaw = 350

# Start SITL with a different port 5762
sitl = SITL()
sitl.download('copter', '3.3', verbose=True)
sitl_args = ['-I0', '--model', 'quad', '--home=50.4507390,30.4612420', '--instance', '0', '--speedup', '1', '--defaults', 'Copter', '--out', 'tcp:127.0.0.1:5762']
sitl.launch(sitl_args, verbose=True)

# Connect to the drone
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    print("Arming and taking off...")
    vehicle.mode = VehicleMode("ALT_HOLD")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    vehicle.mode = VehicleMode("ALT_HOLD")
    print("Taking off to {} meters...".format(aTargetAltitude))
    vehicle.channels.overrides = {'3': 1700}

    while True:
        print("Current altitude: ", vehicle.location.global_relative_frame.alt)

        # Check if the target altitude is reached
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Target altitude reached.")
            break

        time.sleep(1)

    # Save the last channel values
    last_channels = vehicle.channels.overrides

    # Set the last channel values to avoid the override affecting movements
    vehicle.channels.overrides = last_channels

    time.sleep(1)
    print("Takeoff completed.")


def get_bearing(location1, location2):
    d_lat = math.radians(location2.lat - location1.lat)
    d_lon = math.radians(location2.lon - location1.lon)

    y = math.sin(d_lon) * math.cos(math.radians(location2.lat))
    x = math.cos(math.radians(location1.lat)) * math.sin(math.radians(location2.lat)) - \
        math.sin(math.radians(location1.lat)) * math.cos(math.radians(location2.lat)) * math.cos(d_lon)

    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error):
        self.integral += error
        derivative = error - self.prev_error
        correction = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return correction


def goto_point(target_location, pid_roll, pid_pitch, target_height=10):
    print(f"Moving to {target_location} at {target_height} meters...")

    try:
        while True:
            current_location = vehicle.location.global_relative_frame
            current_height = current_location.alt
            current_bearing = get_bearing(current_location, target_location)

            distance = get_distance(current_location, target_location)
            print(" Distance to target: ", distance)
            print(" Current altitude: ", current_height)
            print(" Current bearing to target: ", current_bearing)
            print("Current location ", current_location)
            if distance < 0.01 and abs(target_height - current_height) < 1:
                print(f"Reached the target at {target_height} meters.")
                break

            # Calculate angle error
            angle_error = current_bearing - vehicle.heading

            # Use PID controller for Roll correction
            roll_correction = pid_roll.update(angle_error)

            # Use PID controller for Pitch correction
            pitch_correction = pid_pitch.update(angle_error)

            vehicle.channels.overrides['1'] = int(max(1000, min(2000, 1500 + roll_correction)))
            vehicle.channels.overrides['2'] = int(max(1000, min(2000, 1500 - pitch_correction)))

            # Keep Throttle (channel 3) unchanged
            vehicle.channels.overrides['3'] = 1500

            time.sleep(1)

    finally:
        vehicle.channels.overrides['3'] = 1500
        vehicle.channels.overrides['1'] = 1500
        vehicle.channels.overrides['2'] = 1500


def get_distance(location1, location2):
    coord1 = (location1.lat, location1.lon)
    coord2 = (location2.lat, location2.lon)

    distance = geodesic(coord1, coord2).kilometers

    return distance


def turn_to_azimuth(target_azimuth):
    print(f"Turning to azimuth {target_azimuth} degrees...")

    try:
        while abs(vehicle.heading - target_azimuth) > 5:
            yaw_error = target_azimuth - vehicle.heading
            vehicle.channels.overrides['4'] = int(max(1000, min(2000, 1500 + yaw_error)))
            time.sleep(1)

    finally:
        vehicle.channels.overrides['4'] = {}
        print(f"Reached azimuth {target_azimuth} degrees.")


def land():
    print("Landing...")
    for throttle_value in range(1500, 1400, -10):
        vehicle.channels.overrides['3'] = throttle_value
        time.sleep(1)

    vehicle.close()
    print("Landing completed. Program terminated.")


def set_yaw(azimuth):
    print("Setting yaw to {} degrees...".format(azimuth))
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        azimuth,
        0,
        1,
        0,
        0, 0, 0)
    vehicle.send_mavlink(msg)
    time.sleep(1)


try:
    arm_and_takeoff(altitude)
    target_location = LocationGlobalRelative(point_b[0], point_b[1], altitude)
    goto_point(target_location, pid_roll=PIDController(Kp=0.8, Ki=0.052, Kd=0.7),
              pid_pitch=PIDController(Kp=0.7, Ki=0.045, Kd=0.035))
    set_yaw(azimuth_yaw)

finally:
    print("Returning to point A...")
    target_location_a = LocationGlobalRelative(point_a[0], point_a[1], altitude)
    goto_point(target_location_a, pid_roll=PIDController(Kp=0.8, Ki=0.056, Kd=0.7),
              pid_pitch=PIDController(Kp=0.7, Ki=0.045, Kd=0.035))

    while True:
        distance_a = get_distance(vehicle.location.global_frame, target_location_a)
        print(" Distance to point A: ", distance_a)
        if distance_a < 0.01:
            print("Reached point A.")
            break
        time.sleep(1)

    land()
