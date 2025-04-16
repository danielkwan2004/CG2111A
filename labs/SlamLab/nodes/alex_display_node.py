# This node displays the raw LIDAR scan data and SLAM map in real-time using matplotlib figures. It subscribes to the "lidar/scan" and "slam/mappose" topics to receive LIDAR scan data and SLAM map updates, respectively.

# Import Python Native Modules. We require the Barrier class from threading to synchronize the start of multiple threads. In this case, lidarDisplayProcess is a process not a thread, so we actually use multiprocessing.Barrier instead of threading.Barrier. However, the usage is the same, so we can keep the import as is so that python syntax highlighters don't complain (multiprocessing.Barrier is defined in such a way that syntax highlighters sometimes have trouble).
# from multiprocessing import Barrier
from threading import Barrier

# Import the required pubsub modules. PubSubMsg class to extract the payload from a message.
from pubsub.pub_sub_manager import ManagedPubSubRunnable, PubSubMsg
from pubsub.pub_sub_manager import publish, subscribe, unsubscribe, getMessages, getCurrentExecutionContext

# Import the required modules
# from deprecated.alex_display import LiveDisplayFigure, LidarBasicDisplay,LidarSlamDisplay
from lidar.alex_lidar import resampleLidarScan
from slam.alex_slam import mapBytesToGrid, gridToMapBytes
from display.alex_display_utilities import projectCoordinates,rotateAboutOrigin, getDelta
import numpy as np

# Matplotlib imports 
import matplotlib
matplotlib.use('TKagg')
from matplotlib import gridspec
import matplotlib.pyplot as plt
import matplotlib.cm as colormap
from matplotlib.animation import FuncAnimation


###########################
### Constants and Topics ##
###########################
ARDUINO_SEND_TOPIC = "arduino/send"
SLAM_MAPPOSE_TOPIC = "slam/mappose"
LIDAR_SCAN_TOPIC = "lidar/scan"


# Plotting Costants
FRAMERATE =60
LIDAR_OFFSET_DEGREES = 360
LIDAR_OFFSET_RADIANS = np.deg2rad(LIDAR_OFFSET_DEGREES)

# SLAM constants
MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 8
MAP_SIZE_MILLIMETERS = MAP_SIZE_METERS * 1000
ROBOT_WIDTH_METERS = 0.2 
ROBOT_HEIGHT_METERS = 0.3 

# Coordinate Systems
# Breezy SLAM MAP IMAGE Coordinate System
IMAGE_MAP_PIXELS = MAP_SIZE_PIXELS
IMAGE_MAP_METERS = MAP_SIZE_METERS
METERS_PER_IMAGE_PIXEL = MAP_SIZE_METERS / MAP_SIZE_PIXELS
MILLIMETERS_PER_IMAGE_PIXEL = METERS_PER_IMAGE_PIXEL * 1000



def lidarDisplayProcess(setupBarrier:Barrier=None, readyBarrier:Barrier=None):
    """
    Initializes and runs the Lidar Display Process.
    This function sets up and displays live Lidar scan data and the SLAM map using matplotlib figures.
    It subscribes to the necessary topics, configures the display figures, and updates the data in real-time.
    Additionally, it binds GUI key events for controlling the robot.

    It waits for all required threads to be ready before starting the display loop and ensures a graceful shutdown upon exit.

    The LiveDisplayFigure class is used to create a live display with multiple subplots for Lidar scans and SLAM maps.
    LidarBasicDisplay and LidarSlamDisplay manage the individual plots for the Lidar scan and the SLAM map, respectively.

    Args:
        setupBarrier (Barrier, optional): A threading barrier used for initial setup synchronization. Defaults to None.
        readyBarrier (Barrier, optional): A threading barrier to synchronize the start of the display process with other threads. Defaults to None.
    """

    # Perform any setup here
    ctx:ManagedPubSubRunnable = getCurrentExecutionContext()


    # Perform any setup here
    setupBarrier.wait() if readyBarrier != None else None


    # Subscribing
    subscribe(topic=LIDAR_SCAN_TOPIC, ensureReply=True, replyTimeout=1)
    subscribe(topic=SLAM_MAPPOSE_TOPIC, ensureReply=True, replyTimeout=1)

    # Setting up plots
    fig,gs = createOverallPlot()
    lidarScanAxis, scanPointsArtist = createLidarPlot(fig, gs)
    slamMapAxis, slamMapArtist, robotArtist = createSlamPlot(fig, gs)

    # [Optional] Bind GUI key events to control the robot
    # https://matplotlib.org/stable/gallery/event_handling/keypress_demo.html
    # publish to dedicated topic or directly to arduino/send topic (with the correct payload)


    # Printing Parameters
    paramsStr = f"Display Parameters:\n"
    paramsStr += f"Lidar Offset (Degrees): {LIDAR_OFFSET_DEGREES}\n"
    paramsStr += f"Map Size (Pixels): {MAP_SIZE_PIXELS}\n"
    paramsStr += f"Map Size (Meters): {MAP_SIZE_METERS}\n"
    paramsStr += f"Robot Width (Meters): {ROBOT_WIDTH_METERS}\n"
    paramsStr += f"Robot Height (Meters): {ROBOT_HEIGHT_METERS}\n"
    paramsStr += f"Max Framerate: {FRAMERATE}\n"
    
    print(paramsStr)


    # Wait for all Threads ready
    readyBarrier.wait() if readyBarrier != None else None

    # Display Loop, will not terminate with the context exit flag
    # Instead it will terminate when the user closes the figure, and trigger the context exit flag after that
    # Much like the CLI thread
    try:
        ani = FuncAnimation(fig, updateWrapperForMatplotlib, fargs=(scanPointsArtist, slamMapArtist, robotArtist), interval=1000/FRAMERATE, blit=True, cache_frame_data=False, save_count=None)
        plt.show()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Send Thread Exception: {e}")
        pass

    # Shutdown and exit the thread gracefully
    ctx.doExit()
    print("Exiting Lidar Display Process")
    pass



###########################
### Plottting Functions ###
###########################

def createOverallPlot():
    """
    Create overall matplotlib plot layout for displaying Lidar and SLAM data.

    This function configures the base figure and grid layout with specified dimensions,
    margins, and spacing. It also updates global font settings for consistency across plots.

    Returns:
        tuple: A tuple containing the matplotlib Figure object (fig) and the GridSpec object (gs).
    """
    # Create the base figure
    width = 16
    height = 8
    figsize = (width, height)
    dpi = 50
    fig = plt.figure(figsize=figsize, layout=None, dpi = dpi)

    # create a the grid with gridspec
    # Number of rows and columns for the grid
    nrows = 1   
    ncols = 2

    # Bounding box for the actual plots in the figure (i.e., how big your margins are)
    # Values are expressed as a fraction of the figure width or height
    # Can be None to allow the gridspec to automatically determine the margins
    left = 0.05 # distance from the left side of the figure to the left side of the plots
    right = 0.95 # distance from the right side of the figure to the right side of the plots
    bottom = 0.05 # distance from the bottom of the figure to the bottom of the plots
    top = 0.95 # distance from the top of the figure to the top of the plots

    # Spacing between the plots. Again expressed as a fraction of the figure width or height
    # Can be None to allow the gridspec to automatically determine the spacing
    wspace = 0.4 # horizontal spacing between the plots
    hspace = 0.1 # vertical spacing between the plots

    # Create the gridspec
    gs = gridspec.GridSpec(nrows, ncols, figure=fig, left=left, right=right, bottom=bottom, top=top, wspace=wspace, hspace=hspace)

    # Set global font size
    plt.rcParams.update({'font.size': 24})
    

    return fig,gs


### Lidar Plot Functions ###
def createLidarPlot(fig:plt.Figure, gs:gridspec.GridSpec):
    """
    Create and configure the Lidar plot.

    This function sets up a polar subplot in the provided figure to display Lidar scan data.
    It initializes a scatter plot for the Lidar scan points and configures the polar axes including theta direction, tick intervals, and title.

    Args:
        fig (plt.Figure): The Matplotlib figure object.
        gs (gridspec.GridSpec): The gridspec object for subplot layout.

    Returns:
        tuple: (lidarScanAxis, scanPointsArtist)
            lidarScanAxis (plt.Axes): The polar axes object for the Lidar scan.
            scanPointsArtist (plt.Artist): The scatter plot artist for Lidar scan data.
    """
    # We create an axis/sublot in the figure at the location in the grid
    # Matplotlib actually supports polar plots, so we can use that to display the lidar scan
    # https://matplotlib.org/stable/api/_as_gen/matplotlib.figure.Figure.add_subplot.html#matplotlib.figure.Figure.add_subplot
    ax:plt.Axes = fig.add_subplot(gs[0,0], projection='polar')
    ax.set_theta_direction(-1)
    ax.set_theta_zero_location('N')

    # Scatter Plot Parameters
    # https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.scatter.html#matplotlib.axes.Axes.scatter
    marker = 'o'
    markersize = 4
    color = 'red'
    alpha = 1 # transparency

    # we plot some dummy (empty) data to initialize the plot
    # animated=True helps with performance when updating the plot
    dummy_angles = []
    dummy_distances = []
    scanPointsArtist = ax.scatter(dummy_angles, dummy_distances, marker=marker, s=markersize, c=color, alpha=alpha, animated = True)

    # Other Plot Settings
    title = "Lidar Scan"
    maxium_distance_to_plot = 2000
    
    x_minor_ticks_interval = 5
    x_major_ticks_interval = 45

    y_minor_ticks_interval = 100
    y_major_ticks_interval = 500
        
    ax.set_title(title, pad=20)
    ax.set_rmax(maxium_distance_to_plot)

    ax.set_xticks(np.arange(0, 2*np.pi, np.deg2rad(x_major_ticks_interval)))
    ax.set_xticks(np.arange(0, 2*np.pi, np.deg2rad(x_minor_ticks_interval)), minor=True)
    ax.tick_params(axis='x', pad=20)  # Increase pad to move angular tick labels further out

    ax.set_yticks(np.arange(0, maxium_distance_to_plot, y_major_ticks_interval))
    ax.set_yticks(np.arange(0, maxium_distance_to_plot, y_minor_ticks_interval), minor=True)

    lidarScanAxis = ax
    return lidarScanAxis, scanPointsArtist

def updateLidarPlot(message, scanPointsArtist:plt.Artist):
    """
    Update the Lidar scan plot using data from the received message.

    This function extracts the angle, distance, and quality data from the message payload,
    resamples the Lidar scan data to reduce the number of data points via averaging,
    converts the angles to radians for proper rendering in a polar plot, and updates
    the scatter plot artist accordingly.

    Args:
        message: A Pub/Sub message containing the Lidar scan data payload.
        scanPointsArtist (plt.Artist): The scatter plot artist representing the Lidar scan.

    Returns:
        None. The function updates the provided artist in place.
    """

    # extract the payload from the message
    angleData, distanceData, qualityData = PubSubMsg.getPayload(message)

    # We do some processing to reduce the number for datapoints shown
    offset_degrees = LIDAR_OFFSET_DEGREES
    target_measurements_per_scan = 180
    merge_strategy = np.mean
    fill_value = 99999

    # resample the lidar scan data
    dist, angle = resampleLidarScan(distance=distanceData, angles=angleData,
                                    target_measurements_per_scan=target_measurements_per_scan,
                                    offset_degrees=offset_degrees,
                                    merge_strategy=merge_strategy,
                                    fill_value=fill_value)

    # convert to Radians. Matplotlib uses radians for polar plots
    dist = np.array(dist)
    angle = np.deg2rad(angle)

    # update the scatter plot data
    scanPointsArtist.set_offsets(np.column_stack([angle, dist]))

### SLAM Plot Functions ###
def createSlamPlot(fig:plt.Figure, gs:gridspec.GridSpec):
    """
    Create and configure the SLAM plot.

    This function sets up a subplot in the provided figure to display a SLAM map using imshow.
    It creates an initial dummy grayscale map image, configures the axis ticks to represent meters,
    and adds an arrow representing the robot's current position and orientation.

    Args:
        fig (plt.Figure): The figure object for the Matplotlib plot.
        gs (gridspec.GridSpec): The gridspec object for the subplot configuration

    Returns:
        tuple: (slamMapAxis, slamMapArtist, robotArtist)
            slamMapAxis (plt.Axes): The axes object for the SLAM plot.
            slamMapArtist (plt.Artist): The image artist for the SLAM map.
            robotArtist (plt.Artist): The arrow artist representing the robot's position and orientation.
    """
    # We create an axis/sublot in the figure at the location in the grid
    # Just a normal plot for now
    ax = fig.add_subplot(gs[0, 1])

    # General Parameters
    map_size_pixels = IMAGE_MAP_PIXELS
    map_size_meters = IMAGE_MAP_METERS


    # imshow parameters
    # https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.imshow.html#matplotlib.axes.Axes.imshow
    cmap = colormap.gray # greyscale
    origin = 'lower' # origin at the bottom-left

    # We will use imshow to display the map. Create a square pixel grid with a single color value of 125
    # Pixel Grid Size (map_size_pixels x map_size_pixels)
    dummy = np.full((map_size_pixels, map_size_pixels), 125, dtype=np.uint8)
    slamMapArtist = ax.imshow(dummy, cmap=cmap, origin=origin,vmin=0, vmax=255, animated=True)

    # relable axis to show millimeters
    half_map_size_M= map_size_meters / 2

    tick_major_M_interval = 1
    tick_minor_M_interval = 0.5

    tick_major_px_interval = tick_major_M_interval / METERS_PER_IMAGE_PIXEL
    tick_minor_px_interval = tick_minor_M_interval / METERS_PER_IMAGE_PIXEL

    major_ticks_pixels = np.arange(0, map_size_pixels+1, tick_major_px_interval)
    minor_ticks_pixels = np.arange(0, map_size_pixels+1, tick_minor_px_interval)
    major_ticks_labels = [f"{int((x*METERS_PER_IMAGE_PIXEL) - half_map_size_M)}" for x in major_ticks_pixels]
    # minor_ticks_labels = [f"{((x*METERS_PER_IMAGE_PIXEL) - half_map_size_M):.2f}" for x in minor_ticks_pixels]
    ax.grid(True, which = 'both', linestyle = '-', linewidth = 0.5, color = 'black')
    ax.set_xticks(major_ticks_pixels)
    ax.set_xticks(minor_ticks_pixels, minor=True)
    ax.set_yticks(major_ticks_pixels)
    ax.set_yticks(minor_ticks_pixels, minor=True)

    ax.set_xticklabels(major_ticks_labels)
    # ax.set_xticklabels(minor_ticks_labels, minor=True)
    ax.set_yticklabels(major_ticks_labels)
    # ax.set_yticklabels(minor_ticks_labels, minor=True)

    ax.set_xlabel("X (M)")
    ax.set_ylabel("Y (M)")
    ax.set_xlim(0, map_size_pixels)
    ax.set_ylim(0, map_size_pixels)
    ax.set_title("SLAM Map")



    # Displaying Alex's Current Position
    # We will display Alex's current position on the map as a red arrow
    # https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.arrow.html#matplotlib.axes.Axes.arrow
    arrow_x = 0
    arrow_y = 0
    arrow_dx = 0
    arrow_dy = 0

    head_width = ROBOT_WIDTH_METERS/METERS_PER_IMAGE_PIXEL
    head_length = ROBOT_HEIGHT_METERS/METERS_PER_IMAGE_PIXEL   

    facecolor = 'red'
    edgecolor = 'red'

    robotArtist = ax.arrow(arrow_x, arrow_y, arrow_dx, arrow_dy, head_width=head_width, head_length=head_length, fc=facecolor, ec=edgecolor, animated=True, length_includes_head=True,head_starts_at_zero=True)
    slamMapAxis = ax
    return slamMapAxis, slamMapArtist, robotArtist

def updateSlamPlot(message, slamMapArtist:plt.Artist, robotArtist:plt.Artist):
    """
    Update the SLAM map and robot pose based on a received SLAM map pose message.

    This function decodes the payload from the message, which contains the robot's position (x, y in millimeters),
    orientation (theta in degrees), and the SLAM map bytes. It updates the SLAM map by converting the byte array into a grid
    using 'mapBytesToGrid'. The robot's current pose is then updated by computing the arrow's displacement,
    which represents the robot's position and orientation on the map.

    Args:
       message: Pub/Sub message with payload (x, y, thetaDeg, mapbytes).
        slamMapArtist (plt.Artist): The image artist displaying the SLAM map.
        robotArtist (plt.Artist): The arrow artist representing the robot's pose.

    Returns:
        None. The function updates the provided artists in place.
    """ 
    # get payload
    x, y, thetaDeg, mapbytes = PubSubMsg.getPayload(message)
    # x and y are in millimeters, theta is in degrees

    # First we update the map
    # Convert the byte array to a numpy array
    mapgrid = mapBytesToGrid(mapbytes, MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)

    # Update the image
    slamMapArtist.set_data(mapgrid)

    # Then we update the robot position/pose
    # The arrow represents the robot's position and orientation     
    thetaRad = thetaDeg * np.pi / 180
    arrow_length = ROBOT_HEIGHT_METERS*1000
    # We get a projection in the opposite direction of facing
    dy,dx = getDelta(thetaRad+np.pi, arrow_length)

    # Now we need to convert from meters (breezyslam) to pixels (matplotlib)
    scaleFactor = MILLIMETERS_PER_IMAGE_PIXEL
    x = x / scaleFactor
    y = y / scaleFactor
    dx = dx / scaleFactor
    dy = dy / scaleFactor

    # Update the arrow
    robotArtist.set_data(x=x+dx, y=y+dy, dx=-dx, dy=-dy)


### Animation Functions ###
def updateWrapperForMatplotlib(frame, *args, **kwargs):
    """
    Wrapper function for updating Matplotlib animation frames.

    Retrieves pub/sub messages (blocking with timeout) and uses them to update the Lidar and SLAM plot artists.

    Note: If you use the non-blocking version of getMessages, the contention for the queue can cause the animation to slow down.

    Args:
        frame (int): Animation frame number.
        *args: Positional arguments for plot artists.
        **kwargs: Keyword arguments for plot artists.

    Returns:
        tuple: Updated Matplotlib artists (scanPointsArtist, slamMapArtist, robotArtist).
    """
    pubSubMessages = getMessages(block=True, timeout=1)
    r = updatePlots(pubSubMessages,*args,**kwargs)
    return r

def updatePlots(pubSubMessages, scanPointsArtist:plt.Artist, slamMapArtist:plt.Artist, robotArtist:plt.Artist):
    """
    Process the list of Pub/Sub messages and update the respective plot artists for the Lidar scan and SLAM map.

    Iterates over the messages in reverse order (most recent first) to ensure that only the latest message for each topic is processed.
    It then updates the scanPointsArtist for Lidar scans and both slamMapArtist and robotArtist for the SLAM map and robot pose respectively.

    Args:
        pubSubMessages (list): List of incoming Pub/Sub messages.
        scanPointsArtist (plt.Artist): Matplotlib Artist for the Lidar scan data.
        slamMapArtist (plt.Artist): Matplotlib Artist for the SLAM map data.
        robotArtist (plt.Artist): Matplotlib Artist for the robot's position and orientation.

    Returns:
        tuple: Updated Matplotlib artists (scanPointsArtist, slamMapArtist, robotArtist).
    """

    # we avoid updating the plots if there are no messages
    if len(pubSubMessages) == 0:
        # we dont update any artists
        return (scanPointsArtist, slamMapArtist, robotArtist)
    
    # Start from the most recent message = last message in the list
    # Iterate over the messages in reverse order, update the plot data as needed
    scanUpdated = False
    slamUpdated = False
    updatedArtists = []
    for m in reversed(pubSubMessages):
        m_topic = PubSubMsg.getTopic(m)
        if (m_topic == LIDAR_SCAN_TOPIC) and (not scanUpdated):
            updateLidarPlot(m, scanPointsArtist)
            scanUpdated = True
            updatedArtists.append(scanPointsArtist) 
        elif (m_topic == SLAM_MAPPOSE_TOPIC) and (not slamUpdated):
            updateSlamPlot(m, slamMapArtist, robotArtist)
            slamUpdated = True
            updatedArtists.append(slamMapArtist)
            updatedArtists.append(robotArtist)
        if scanUpdated and slamUpdated:
            # we have updated both plots, so we can break out of the loop
            # Since all further messages will be older than the ones we have already processed
            break
    return (scanPointsArtist, slamMapArtist, robotArtist)
            

    
