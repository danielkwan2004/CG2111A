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

# Bokeh Imports
from bokeh.plotting import figure, curdoc
from bokeh.models import ColumnDataSource
from bokeh.server.server import Server
from bokeh.application import Application
from bokeh.application.handlers.function import FunctionHandler
from bokeh.layouts import row
import time

# Hack to enable compression in Bokeh WebSocket handler
# Comment to disable compression
from bokeh.server.views.ws import WSHandler
import bokeh.server.views.ws
class CompressionEnabledWSHandler(WSHandler):
    def get_compression_options(self):
        # Return a non-None value (an empty dict enables default compression)
        return {"compression_level": 9}
bokeh.server.views.ws.WSHandler = CompressionEnabledWSHandler




###########################
### Constants and Topics ##
###########################
ARDUINO_SEND_TOPIC = "arduino/send"
SLAM_MAPPOSE_TOPIC = "slam/mappose"
LIDAR_SCAN_TOPIC = "lidar/scan"


# Plotting Costants
FRAMERATE =60
LIDAR_OFFSET_DEGREES = 360 # should match whatever is set in the SLAM node
LIDAR_OFFSET_RADIANS = np.deg2rad(LIDAR_OFFSET_DEGREES)

# SLAM and MAP constants
MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 8
MAP_SIZE_MILLIMETERS = MAP_SIZE_METERS * 1000
ROBOT_WIDTH_METERS = 0.2 
ROBOT_HEIGHT_METERS = 0.2

ROBOT_HALF_HEIGHT_MM = ROBOT_HEIGHT_METERS * 1000 / 2
ROBOT_HALF_WIDTH_MM = ROBOT_WIDTH_METERS * 1000 / 2
ROBOT_HALF_DIAGONAL = np.sqrt(ROBOT_HALF_HEIGHT_MM**2 + ROBOT_HALF_WIDTH_MM**2)

IMAGE_MAP_DOWNSCALE_FACTOR = 1 # Downscale the image by this factor for display purposes
SLAM_MAP_GUI_UPDATE_INTERVAL = 0.25 # seconds, minimum time between updates to the SLAM map image



def lidarDisplayProcess(setupBarrier:Barrier=None, readyBarrier:Barrier=None):
    """
    Initializes and runs the Lidar Display Process.
    This function sets up and displays live LIDAR scan data and the SLAM map.

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
    # Create the Bokeh plot and data source
    datasources = {}
    lidarPlot, lidarDs = createLidarPlot()
    datasources["lidarscan"] = lidarDs

    slamPlot, imageDs, poseDs = createSlamPlot()
    datasources["slam"] = {
        "image": imageDs,
        "pose": poseDs
    }

    overallPlot = createLayout([lidarPlot, slamPlot])


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

    # Create the Bokeh server
    serv = setupBokehServer(overallPlot, datasources)


    # Wait for all Threads ready
    readyBarrier.wait() if readyBarrier != None else None

    # Display Loop, will not terminate with the context exit flag
    # Instead it will terminate when the user closes the figure, and trigger the context exit flag after that
    # Much like the CLI thread
    try:
        runBokehServer(serv) # Start the Bokeh server in the main thread

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Send Thread Exception: {e}")
        pass

    # Shutdown and exit the thread gracefully
    ctx.doExit()
    print("Exiting Lidar Display Process")
    pass

def polarToCartesian(angles, distances, cardinalZero="N"):
    """
    Convert polar coordinates to Cartesian coordinates.

    Args:
        angle (float): Angle in radians.
        distance (float): Distance from the origin.

    Returns:
        tuple: x and y coordinates in Cartesian system.
    """ 

    angles = -np.array(angles) # Invert the angles to match the polar coordinate system
    distances = np.array(distances)

    if cardinalZero == "N":
        # use North as 0 degrees
        angles = angles + 90
    elif cardinalZero == "E":
        # use East as 0 degrees
        angles = angles - 0
    elif cardinalZero == "S":
        # use South as 0 degrees
        angles = angles  - 90
    elif cardinalZero == "W":
        # use West as 0 degrees
        angles = angles - 180

    angleRad = np.deg2rad(angles)
    x = distances * np.cos(angleRad)
    y = distances * np.sin(angleRad)
    return x, y


###########################
### Plottting Functions ###
###########################

def makeRobotTriangle(
        robotOriginXmm, robotOriginYmm, 
        robotThetaDegrees=0, #from north
        ):
    """
    Generate coordinates for a triangle representing the robot.

    Args:
        robotOriginXmm (float): The X coordinate of the robot's origin in millimeters.
        robotOriginYmm (float): The Y coordinate of the robot's origin in millimeters.
        robotThetaDegrees (float, optional): The robot's heading in degrees (from north). Defaults to 0.

    Returns:
        tuple: Two lists containing the X and Y coordinates of the triangle vertices.
    """


    # We need to convert to bokehs angles 
    # In bokeh
    # cw is negative, ccw is positive
    # zero degrees is at the top (east)
    front = -robotThetaDegrees + 90
    angle_bottom_left = front + 90 + 45
    angle_bottom_right = front - 90 - 45

    # convert to radians
    front = np.deg2rad(front)
    angle_bottom_left = np.deg2rad(angle_bottom_left)
    angle_bottom_right = np.deg2rad(angle_bottom_right)


    top = projectCoordinates(robotOriginXmm, robotOriginYmm, front, ROBOT_HALF_HEIGHT_MM)
    bottom_left = projectCoordinates(robotOriginXmm, robotOriginYmm, angle_bottom_left, ROBOT_HALF_DIAGONAL)
    bottom_right = projectCoordinates(robotOriginXmm, robotOriginYmm, angle_bottom_right, ROBOT_HALF_DIAGONAL)

    robot_Xs = [top[0], bottom_left[0], bottom_right[0]]
    robot_Ys = [ top[1], bottom_left[1], bottom_right[1]]
    
    return robot_Xs, robot_Ys


def createLayout(plots):
    """
    Arrange the provided Bokeh plots in a horizontal row.

    Args:
        plots (list): List of Bokeh plot objects.

    Returns:
        row: A Bokeh row layout containing the plots.
    """
    layout = row(*plots)
    return layout

def createLidarPlot():
    """
    Create a Bokeh plot to display the Lidar scan data.

    Returns:
        tuple: A tuple containing the Bokeh figure and its associated ColumnDataSource.
    """

    # Create a Bokeh figure to display the Lidar scan data
    # https://docs.bokeh.org/en/latest/docs/reference/plotting/figure.html
    p = figure(title="Lidar Scan", x_axis_label='X (mm)', y_axis_label='Y (mm)')
    
    # set the size of the figure
    p.width = 800
    p.height = 800

    # Create a ColumnDataSource to hold the data
    source = ColumnDataSource(data=dict(x=[], y=[]))

    # Add a scatter renderer to the plot
    pointSize = 3
    pointColor = "red"
    pointAlpha = 1
    legendLabel = "Lidar Scan"
    # the types of markers are
    # 'circle', 'square', 'triangle', 'diamond', 'cross', 'x'
    pointMarker = "circle"
    p.scatter(x='x', y='y', source=source, size=pointSize, color=pointColor, alpha=pointAlpha, legend_label=legendLabel, marker=pointMarker)


    # Configure the plot
    # Make sure it does not autoscale the axes
    p.xaxis.axis_label = "X (mm)"
    p.yaxis.axis_label = "Y (mm)"
    p.xaxis.axis_label_standoff = 10

    # Set the x and y axis limits
    # map size/2
    p.x_range.start = -MAP_SIZE_MILLIMETERS / 2
    p.x_range.end = +MAP_SIZE_MILLIMETERS / 2

    p.y_range.start = -MAP_SIZE_MILLIMETERS / 2
    p.y_range.end = +MAP_SIZE_MILLIMETERS / 2


    # Remove ticks
    p.xaxis.ticker = []
    p.yaxis.ticker = []

    # remove axis lines
    p.xaxis.axis_line_color = None
    p.yaxis.axis_line_color = None
    
    # Draw a red triangle at the origin to represent the robot
    # use a patch to draw the triangle
    robot_Xs, robot_Ys = makeRobotTriangle(0, 0, 0)
    robot_color = "red"
    robot_line_color = "black"
    robot_alpha = 1
    p.patch(x=robot_Xs, y=robot_Ys, fill_color=robot_color, line_color=robot_line_color, alpha=robot_alpha)

    # draw draw the distance lines
    firstRing_radius = 500
    ring_interval = 500
    max_distance = 10001
    for i in range(firstRing_radius, max_distance, ring_interval):
        circle = p.circle(x=0, y=0, radius=i, fill_color=None, line_color="black", line_width=0.5, alpha=0.5)
        # add annotation to the circle at 45 degrees
        angle = np.deg2rad(45)
        x = i * np.cos(angle)
        y = i * np.sin(angle)

        # add text to the circle, makes sure to offset the text by 10 pixels
        x_offset = 10 * np.cos(angle)
        y_offset = 10 * np.sin(angle)

        p.text(x=x+x_offset, y=y+y_offset, text=[f"{i} mm"], text_color="black", text_font_size="10pt")
    
    return p, source

def updateLidarPlot(message, datasources):
    """
    Update the Lidar plot with new scan data from a PubSub message.

    This function extracts angle, distance, and quality data from the message payload,
    filters out low-quality measurements, resamples the LIDAR scan to a fixed number of measurements,
    converts the polar coordinates to Cartesian coordinates, and streams the resulting data
    to the associated Bokeh ColumnDataSource.

    Args:
        message: A PubSubMsg containing the LIDAR scan data payload.
        datasources (dict): A dictionary containing the Bokeh data sources; it must include the key "lidarscan".

    Returns:
        None
    """

    angleData, distanceData, qualityData = PubSubMsg.getPayload(message)

    goodQuality = np.array(qualityData) > 100
    angleData = np.array(angleData)[goodQuality]
    distanceData = np.array(distanceData)[goodQuality]

    offset_degrees = LIDAR_OFFSET_DEGREES
    target_measurements_per_scan = 180
    merge_strategy = np.mean
    fill_value = 99999

    dist, angle = resampleLidarScan(distance=distanceData, angles=angleData,
                    target_measurements_per_scan=target_measurements_per_scan,
                    offset_degrees=offset_degrees,
                    merge_strategy=merge_strategy,
                    fill_value=fill_value)

    # to cartesian coordinates
    x, y = polarToCartesian(angle, dist)
    rollover = len(x)
    datasources["lidarscan"].stream({'x': x, 'y': y}, rollover=rollover)
    return 

def createSlamPlot():
    """
    Create a Bokeh plot to display the SLAM map and the robot's pose.

    Returns:
        tuple: A tuple containing the Bokeh figure, an image ColumnDataSource, and a pose ColumnDataSource.
    """

    p = figure(title="SLAM Map", x_axis_label='X (M)', y_axis_label='Y (M)')

    # set the size of the figure
    p.width = 800
    p.height = 800

    # Create a ColumnDataSource to hold the data
    # datasource is an image
    imageSource = ColumnDataSource(data=dict(image=[]))
    poseSource = ColumnDataSource(data=dict(px=[], py=[]))

    # Add a image render
    map_zero_x = 0
    map_zero_y = 0
    map_w_mm = MAP_SIZE_METERS * 1000 
    map_h_mm = MAP_SIZE_METERS * 1000 

    p.image(image='image',  x=map_zero_x, y=map_zero_y, dw=map_w_mm, dh=map_h_mm, source=imageSource, palette="Greys256")

    # add a patch to represent the robot
    p.patch(x='px', y='py', source=poseSource, fill_color="red", line_color="black", alpha=1, level = "glyph")

    # Configure the plot
    return p, imageSource, poseSource

lastUpdateSlamPicture = [0] # little hack to create a static/global mutable variable
def updateSlamPlot(message, datasources):
    """
    Update the SLAM plot with new SLAM map and robot pose data.

    This function extracts the robot's x, y coordinates, heading (theta in degrees),
    and the map bytes from the provided PubSub message. It converts the map bytes into a grid,
    generates the robot triangle for visualization, and updates the corresponding Bokeh data
    sources ("image" for the map and "pose" for the robot's position).

    Args:
        message: A PubSubMsg instance containing the SLAM map and pose payload.
        datasources (dict): Dictionary containing the Bokeh data sources with the following structure:
            {
                "slam": {
                    "image": ColumnDataSource for map image,
                    "pose": ColumnDataSource for robot pose
                }
            }

    Returns:
        None
    """


    # Extract the payload from the message 
    # The payload is a map and a pose
    x, y, thetaDeg, mapbytes = PubSubMsg.getPayload(message)
    
    # Convert the map data to a grid
    grid = mapBytesToGrid(mapbytes, MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)

    # Make the robot triangle
    Xs, Ys = makeRobotTriangle(x, y, thetaDeg)

    

    # Update the image source with the new map image
    currentTime = time.time()
    if (currentTime - lastUpdateSlamPicture[0]) > SLAM_MAP_GUI_UPDATE_INTERVAL:
        lastUpdateSlamPicture[0] = currentTime
        # Update the image source with the new map image
        # Downscale as needed
        grid = grid[::IMAGE_MAP_DOWNSCALE_FACTOR, ::IMAGE_MAP_DOWNSCALE_FACTOR]
        datasources["slam"]["image"].data = dict(image=[grid])

    # Update the pose source with the new robot pose
    datasources["slam"]["pose"].data = dict(px=Xs, py=Ys)
    return
    
def updatePlots(datasources):
    """
    Process incoming PubSub messages and update Bokeh plots accordingly.

    This function polls for new messages from the PubSub message queue and updates the
    LIDAR scan and SLAM map plots. It processes messages in reverse order, and for each relevant
    topic (LIDAR scan and SLAM map/pose), it calls the designated update functions once per display
    refresh.

    Args:
        datasources (dict): A dictionary containing Bokeh ColumnDataSource objects for the plots. It must include:
            - "lidarscan": ColumnDataSource for the LIDAR scan plot.
            - "slam": A dictionary with keys "image" (for the SLAM map) and "pose" (for the robot pose patch).

    Returns:
        None
    """

    pubSubMessages = getMessages(block=True, timeout=1)
    updates = {x:False for x in datasources.keys()}
    for m in reversed(pubSubMessages):
        m_topic = PubSubMsg.getTopic(m)
        if (m_topic == LIDAR_SCAN_TOPIC) and (not updates["lidarscan"]):
            # Extract the payload from the message 
            updateLidarPlot(m, datasources)
            updates["lidarscan"] = True
        elif (m_topic == SLAM_MAPPOSE_TOPIC) and (not updates["slam"]):
            # Extract the payload from the message 
            updateSlamPlot(m, datasources)
            updates["slam"] = True
            
        if all(updates.values()):
            break



def setupBokehServer(bokehPlot, datasources):
    """
    Set up the Bokeh server.

    This function creates a Bokeh application with the provided layout, registers periodic callbacks to update the plots
    using the supplied data sources, and starts the Bokeh server.

    Args:
        bokehPlot: A Bokeh layout object containing the plotting elements.
        datasources (dict): Dictionary of Bokeh ColumnDataSource objects used for updating the plots.

    Returns:
        server: The Bokeh Server instance.
    """

    def initPlots(doc):
        # Initialize the plot with empty data
        doc.add_root(bokehPlot)
        doc.add_periodic_callback(lambda: updatePlots(datasources), 1000 / FRAMERATE)
    app = Application(FunctionHandler(initPlots))

    ip = "0.0.0.0"
    port = 8181
    server = Server({'/': app}, 
                    address=ip, port=port,
                    allow_websocket_origin=[f"*:{port}"],
                    )
    server.start()
    print(f"Bokeh server is running at http://{ip}:{port}/")
    if ip == "0.0.0.0":
        print(f"{ip} means all interfaces. Access it from your host machine using the IP address of your Pi.")

    return server

def runBokehServer(server):
    """
    Run the Bokeh server.
    
    Starts the server's I/O loop that serves the Bokeh application. This call blocks until the server shuts down.
    
    Args:
        server: A Bokeh Server instance.
    
    Returns:
        None
    """

    server.io_loop.start()

