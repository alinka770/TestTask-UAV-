from dronekit_sitl import SITL
from dronekit import connect, Vehicle, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil
import time

# Coordinates for points A and B
point_a = (50.4507390, 30.4612420)
point_b = (50.4433260, 30.4480780)

# Flight altitude (in meters)
altitude = 100

# Yaw angle (in degrees)
azimuth_yaw = 350

# Start SITL with a different port 5762
sitl = SITL()
sitl.download('copter', '3.3', verbose=True)
sitl_args = ['-I0', '--model', 'quad','--instance', '0', '--speedup', '1', '--defaults', 'Copter', '--out', 'tcp:127.0.0.1:5762']
sitl.launch(sitl_args, verbose=True)

# Connect to the vehicle with the specified port
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, vehicle_class=Vehicle, timeout=60)


# Takeoff
def takeoff(target_altitude):
    print("Taking off to {} meters...".format(target_altitude))
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude*0.999:
            print("Reached target altitude")
            break
        time.sleep(1)

# Set constant altitude during flight
def set_altitude(target_altitude):
    print("Setting altitude to {} meters...".format(target_altitude))
    vehicle.simple_goto(LocationGlobalRelative(
        vehicle.location.global_frame.lat,
        vehicle.location.global_frame.lon,
        target_altitude))

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if abs(vehicle.location.global_relative_frame.alt - target_altitude) < 0.5:
            print("Altitude stabilized")
            break
        time.sleep(1)

# Go to point B
def goto_point_b(target_location):
    print("Going to point B...")
    vehicle.simple_goto(target_location)

    while True:
        distance = get_distance(vehicle.location.global_frame, target_location)
        print(" Distance to target: ", distance)
        if distance < 1:
            print("Reached point B")
            break
        time.sleep(1)

# Set yaw angle
def set_yaw(azimuth):
    print("Setting yaw to {} degrees...".format(azimuth))
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0, #confirmation
        azimuth,    # param 1, yaw in degrees
        0,         
        1,          
        0, 
        0, 0, 0)    
    # send command to vehicle
    vehicle.send_mavlink(msg)
    time.sleep(1)

# Get distance between two points
def get_distance(location1, location2):
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return (dlat**2 + dlong**2)**0.5 * 1.113195e5

try:
    # Take off
    takeoff(altitude)

    # Set constant altitude during flight
    set_altitude(altitude)

    # Go to point B
    target_location = LocationGlobalRelative(point_b[0], point_b[1], altitude)
    goto_point_b(target_location)

    # Set yaw angle
    set_yaw(azimuth_yaw)

finally:
    # Return to point A
    print("Returning to point A...")
    target_location_a = LocationGlobalRelative(point_a[0], point_a[1], altitude)
    vehicle.simple_goto(target_location_a)

    # Wait for reaching point A
    while True:
        distance_a = get_distance(vehicle.location.global_frame, target_location_a)
        print(" Distance to point A: ", distance_a)
        if distance_a < 1:
            print("Reached point A")
            break
        time.sleep(1)

    # Landing
    print("Landing...")
    vehicle.armed = False
    time.sleep(5)
    vehicle.mode = VehicleMode("LAND")
    vehicle.close()
    print("Mission completed")
