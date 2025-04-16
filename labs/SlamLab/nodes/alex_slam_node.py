# This node subscribes to the "lidar/scan" topic to receive LIDAR scan data, processes the scans, updates the SLAM algorithm, and publishes the resulting map and robot position.

# Import Python Native Modules. We require the Barrier class from threading to synchronize the start of multiple threads.
from threading import Barrier

# Import the required pubsub modules. PubSubMsg class to extract the payload from a message.
from pubsub.pub_sub_manager import ManagedPubSubRunnable, PubSubMsg
from pubsub.pub_sub_manager import publish, subscribe, unsubscribe, getMessages, getCurrentExecutionContext

# Import the required slam modules
from slam.alex_slam import RMHC_SLAM, Laser, getGridAlignedSlamPose
from lidar.alex_lidar import resampleLidarScan


# Constants
# Slam Constants, Slam is senstive to these values
# Changing these values might significantly break the slam algorithm
# Modify with caution
NUMBER_OF_DATAPOINTS_PER_ROUND = 360
SCANNING_FREQUENCY = 2
SCANNING_FIELD_OF_VIEW = 360
DISTANCE_IF_NO_OBSTACLE = 12000
HOLE_WIDTH_MM = 100
MIN_SAMPLES_FOR_SLAM_UPDATE = 200


# Map Constants
MAP_QUALITY = 5
MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 8
MAP_SIZE_MILLIMETERS = MAP_SIZE_METERS * 1000
LIDAR_OFFSET_DEGREES = 360 # in case your lidar 'Front' (i.e. 0 degrees) is not aligned with the robot 

LIDAR_SCAN_TOPIC = "lidar/scan"
SLAM_MAPPOSE_TOPIC = "slam/mappose"
TRAJECTORY_TOPIC = "slam/trajectory"


def slamThread(setupBarrier:Barrier=None, readyBarrier:Barrier=None):
    """
    SLAM (Simultaneous Localization and Mapping) thread function.
    This function initializes and runs a SLAM algorithm using LIDAR data. It subscribes to LIDAR scan messages,
    processes the scans, updates the SLAM algorithm, and publishes the resulting map and robot position.
    
    Args:
        setupBarrier (Barrier): A barrier to synchronize the setup of the SLAM algorithm.
        readyBarrier (Barrier): A barrier to synchronize the readiness of the SLAM thread.

    The function performs the following steps:
    1. Initializes the SLAM algorithm with specified parameters.
    2. Subscribes to the "lidar/scan" topic to receive LIDAR scan data.
    3. Waits for all threads to be ready if a readyBarrier is provided.
    4. Enters a loop where it:
        - Retrieves the latest LIDAR scan message.
        - Processes the scan data (resampling and filtering).
        - Updates the SLAM algorithm with the processed scan data.
        - Retrieves the current robot position and map.
        - Publishes the map and robot position to the "slam/mappose" topic.
    5. Gracefully exits the thread when the execution context signals an exit.
    
    Note:
        - The function assumes the existence of several helper functions and classes such as `getCurrentExecutionContext`,
          `RMHC_SLAM`, `Laser`, `subscribe`, `getMessages`, `PubSubMsg`, `resampleLidarScan`, and `publish`.
        - The function uses a blocking call to `getMessages` with a timeout of 1 second to retrieve LIDAR scan messages.
        - The SLAM algorithm is updated only if the number of valid distance measurements exceeds `MIN_SAMPLES_FOR_SLAM_UPDATE`.
        - If the current scan is inadequate, the previous scan data is used to update the SLAM algorithm.
    """

    # Perform any setup here
    ctx:ManagedPubSubRunnable = getCurrentExecutionContext()


    # Set up the SLAM algorithm
    setupBarrier.wait() if readyBarrier != None else None


    # Initialize an empty trajectory. This is UNUSED in this implementation
    # You may use this to store the robot's trajectory (i.e., the series of positions the robot has been in)
    trajectory = []

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles    = None

    # subscribe to lidar scan
    subscribe(topic=LIDAR_SCAN_TOPIC, ensureReply=True, replyTimeout=1)


    # Print the SLAM parameters
    slam, slamParamString = makeSlam()
    print(slamParamString)


    # Wait for all Threads ready
    readyBarrier.wait() if readyBarrier != None else None


    # Implement SLAM logic here
    try:
        while(not ctx.isExit()):
            # get the lastest lidar scan
            messages = getMessages(block=True, timeout=1)
            if not messages:
                continue

            # get the most recent message and discard the rest
            # If we are producing data faster than we can consume
            # This ensures we don't fall behind
            message = messages[-1]
            # get the payload
            ang, dist, quality = PubSubMsg.getPayload(message)

            # [OPTIONAL] Filter the scan data to make slam better!
            # bad quality scans are not good for slam

            # Resample the lidar scan to to fit the slam parameters
            # We also rotate the scan by 90 degrees to align with the robot
            dist, ang = resampleLidarScan(dist, ang, target_measurements_per_scan=360, offset_degrees=LIDAR_OFFSET_DEGREES, fill_value=12000)


            # Update SLAM if the current scan is adequate
            if len(dist) > MIN_SAMPLES_FOR_SLAM_UPDATE:
                # print("Updating SLAM")
                slam.update(dist, scan_angles_degrees=ang)
                previous_distances = dist.copy()
                previous_angles    = ang.copy()


            # If not adequate, use previous
            elif previous_distances is not None:
                # print("Updating SLAM Previous")
                slam.update(previous_distances, scan_angles_degrees=previous_angles)

            # Get current robot position
            x, y, theta = slam.getpos()
            x, y, theta = getGridAlignedSlamPose(x, y, theta, MAP_SIZE_MILLIMETERS)


            # Consider Updating trajectory here?
            # trajectory.append((x, y, theta))
        

            # Get current map bytes as grayscale
            # This writes the current slam map into the mapbytes bytearray
            slam.getmap(mapbytes)

            # publish the map and the robot position
            publish(SLAM_MAPPOSE_TOPIC, (x, y, theta, mapbytes.copy()))

            # publish the trajectory?
            # publish(TRAJECTORY_TOPIC, tuple(trajectory))
    
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"SLAM Thread Exception: {e}")
        pass


    # Shutdown and exit the thread gracefully
    ctx.doExit()
    print("Exiting SLAM Thread")
    pass

### Helper Functions ###
def makeSlam():
    """
    Initializes the SLAM algorithm with specified parameters.
    """
    min_samples = MIN_SAMPLES_FOR_SLAM_UPDATE # Minimum number of samples per scan to update SLAM
    map_size_pixels = MAP_SIZE_PIXELS # length of one side of the square map in pixels
    map_size_meters = MAP_SIZE_METERS # corresponding length of one side of the square map in meters
    hole_width_mm = HOLE_WIDTH_MM # how large a "hole" or gap we can have between two points to still consider them connected, in mm
    map_quality = MAP_QUALITY # quality of the map, determins how quickly new scans are integrated into the exisitng map
    laser = Laser(NUMBER_OF_DATAPOINTS_PER_ROUND, SCANNING_FREQUENCY, SCANNING_FIELD_OF_VIEW, DISTANCE_IF_NO_OBSTACLE)
    slam = RMHC_SLAM(laser, map_size_pixels, map_size_meters, hole_width_mm=hole_width_mm, map_quality=map_quality)


    slamParamString = "SLAM Parameters:\n"
    slamParamString += f"Number of Data Points per Round: {NUMBER_OF_DATAPOINTS_PER_ROUND}\n"
    slamParamString += f"Scanning Frequency: {SCANNING_FREQUENCY}\n"
    slamParamString += f"Scanning Field of View: {SCANNING_FIELD_OF_VIEW}\n"
    slamParamString += f"Distance if No Obstacle: {DISTANCE_IF_NO_OBSTACLE}\n"
    slamParamString += f"Minimum Samples: {min_samples}\n"
    slamParamString += f"Map Size (Pixels): {map_size_pixels}\n"
    slamParamString += f"Map Size (Meters): {map_size_meters}\n"
    slamParamString += f"Hole Width (mm): {hole_width_mm}\n"
    slamParamString += f"Map Quality: {MAP_QUALITY}\n"
    slamParamString += f"Lidar Offset (Degrees): {LIDAR_OFFSET_DEGREES}\n"

    return slam, slamParamString

