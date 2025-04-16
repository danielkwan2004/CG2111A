"""
This module provides classes for visualizing LIDAR data using Matplotlib.
Classes:
    LiveDisplayFigure: Manages the live display of LIDAR data.
    PubSubDisplayTemplate: Abstract base class for creating custom LIDAR displays.
    LidarBasicDisplay: Displays basic LIDAR scan data in a polar plot.
    LidarSlamDisplay: Displays SLAM (Simultaneous Localization and Mapping) data.
    LidarBasicDisplay2: Alternative implementation for displaying basic LIDAR scan data.
Constants:
    LIDAR_DISPLAY_STRING: A string containing text instructions for controlling the LIDAR display. This string is displayed in the control area of the LidarBasicDisplay2 class for user reference.   
"""


import matplotlib
matplotlib.use('TKagg')

import matplotlib.pyplot as plt
from matplotlib import axes
import matplotlib.cm as colormap
import matplotlib.animation as animation
from abc import ABC, abstractmethod
import numpy as np
import time

LIDAR_DISPLAY_STRING = '''
Welcome to the RPLidar A1M8 Full Scan Display
W -> Move Forward
S -> Move Backward
A -> Turn Left
D -> Turn Right
E -> Stop
Q -> Quit
    '''


class LiveDisplayFigure():
    """
    A class to create and manage a live display figure using matplotlib. 
    
    These class is meant to be instantiated in its own process. It is used to create a figure with multiple subplots, each of which can be updated with new data from a PubSubInterface. The figure can be animated to display the data in real-time. It also provides a method to hook keyboard events to the figure for user interaction, so that the user perform actions (i.e., publish messages) when certain keys are pressed.

    Each subplot is a subclass of PubSubDisplayTemplate, which defines the interface for creating custom displays. The LiveDisplayFigure class manages the creation and updating of these subplots, while the subplots themselves handle the configuration and updating of the individual plots.
    
    Refer to the alex_example_lidar.py file for an example of how to use these classes.


    Attributes:
    -----------
    getMessages : callable
        A function to retrieve messages for updating the plots.
    title : str, optional
        The title of the figure (default is 'Live Display').
    figsize : tuple, optional
        The size of the figure (default is (8, 8)).
    fig_kwargs : dict, optional
        Additional keyword arguments for the figure (default is {}).
    nrows : int, optional
        Number of rows in the grid specification (default is 1).
    ncols : int, optional
        Number of columns in the grid specification (default is 1).
    gridspec_kwargs : dict, optional
        Additional keyword arguments for the grid specification (default is {}).
    target_framerate : int, optional
        The target framerate for updating the display (default is 30).
    Methods:
    --------
    __init__(self, getMessages, title='Live Display', figsize=(8, 8), fig_kwargs={}, nrows=1, ncols=1, gridspec_kwargs={}, target_framerate=30):
        Initializes the LiveDisplayFigure with the given parameters.
    update(self, frame, extra):
        Updates the figure with new data from getMessages.
    animate(self, cache_frame_data=False, save_count=5, extra={}):
        Starts the animation of the figure.
    add_plot(self, plot, idx):
        Adds a plot to the figure.
    hookKeyboardEvents(self, keys, handler):
        Hooks keyboard events to the figure.
    """

    def __init__(self, getMessages,
                title='Live Display', figsize=(8, 8), fig_kwargs={},
                nrows=1, ncols=1, gridspec_kwargs={},
                target_framerate=30
                ): 
        """
        Initializes the Live Display. 

        Args:
            getMessages (callable): A function to retrieve messages for display.
            title (str, optional): The title of the display window. Defaults to 'Live Display'.
            figsize (tuple, optional): The size of the figure in inches. Defaults to (8, 8).
            fig_kwargs (dict, optional): Additional keyword arguments for the figure. Defaults to {}.
            nrows (int, optional): Number of rows in the grid layout. Defaults to 1.
            ncols (int, optional): Number of columns in the grid layout. Defaults to 1.
            gridspec_kwargs (dict, optional): Additional keyword arguments for the grid specification. Defaults to {}.
            target_framerate (int, optional): The target frame rate for the display. Defaults to 30.

        Attributes:
            innerfig (matplotlib.figure.Figure): The main figure for the display.
            innerGS (matplotlib.gridspec.GridSpec): The grid specification for the figure layout.
            managedPlots (list): A list to manage the plots in the display.
            target_framerate (int): The target frame rate for the display.
            getMessages (callable): A function to retrieve messages for display.
        """
        self.innerfig = plt.figure(layout=None, figsize=figsize)
        self.innerGS = plt.GridSpec(nrows, ncols, figure=self.innerfig)
        self.managedPlots = []
        self.target_framerate = target_framerate
        self.getMessages = getMessages  

        self.updateTime = 0

    def update(self, frame, extra):
        """
        Update the display with the given frame and extra data. This function is meant to be called by the animation function. At base level, it retrieves messages from the PubSubInterface and calls the update function of each of the relevant managed subplot with the new data.

        Parameters:
        frame (int): The current frame number.
        extra (dict): Additional data to be used during the update.

        Returns:
        tuple: A tuple containing the updated axes from the managed plots.
        """
        # get messages
        data = self.getMessages(block=True, timeout=1)

        # data = self.getMessages(block=False)
        if not data:
            data = []
        # print(f"Frame: {frame} | Extra: {extra} | ManagedPlots: {self.managedPlots}")
        start= time.perf_counter()
        axs = sum([p.update(frame, data, extra) for p in self.managedPlots if not (p==None)], ())
        end = time.perf_counter()
        
        # priint update time in ms
        print(f"Update Time: {(end-start)*1000:.2f}ms")
        return axs
    
    def animate(self, cache_frame_data=False, save_count=5, extra={}):
        """
        Animates the figure using the specified parameters. This is a wrapper around the matplotlib FuncAnimation function, and will periodically call the update function to update the display. This function shoule be called after adding all the plots to the display, and will block until the gui display is closed.

        Parameters:
        -----------
        cache_frame_data : bool, optional
            Whether to cache frame data. Default is False.
        save_count : int, optional
            The number of frames to cache. Default is 5.
        extra : dict, optional
            Additional arguments to pass to the update function. Default is an empty dictionary.

        Returns:
        --------
        None
        """
        interval = 1000/self.target_framerate
        self.ani = animation.FuncAnimation(fig=self.innerfig, func=self.update, fargs=(extra,),
                                           interval=interval, cache_frame_data=cache_frame_data, save_count=save_count, blit=True)
        plt.show()

    def add_plot(self, plot, idx):
        """
        Adds a plot to the display and initializes it. Plots must be subclasses of PubSubDisplayTemplate.

        Parameters:
        plot (PubSubDisplayTemplate): The plot object to be added.
        idx (int): The index position for the plot in the grid.

        Returns:
        None
        """
        plot:PubSubDisplayTemplate = plot
        ax = plot.initPlot(self.innerfig, self.innerGS, idx)
        self.managedPlots.append(plot)


    def hookKeyboardEvents(self, keys, handler):
        """
        Hook keyboard events to a specified handler function.
        This method connects a handler function to specific keyboard events
        defined in the `keys` dictionary. When a key event occurs, the handler
        function is called with the corresponding value from the `keys` dictionary.
        Parameters:
        keys (dict): A dictionary where the keys are the keyboard keys to listen for,
                     and the values are the corresponding actions or data to pass to the handler.
        handler (function): A function to handle the keyboard events. It should accept a single
                            argument which will be the value from the `keys` dictionary.
        Returns:
        int: The connection id (cid) of the event handler, which can be used to disconnect
             the event later if needed.
        """

        # Remove the default save key binding, since it conflicts with the custom key bindings
        # Users can remove this line if they want to keep the save key binding
        # or remove other key bindings as needed
        # Refer to matplotlib documentation to find the keymap names
        plt.rcParams['keymap.save'].remove('s')

        def on_kp_event(event):
            if event.key in keys:
                handler(keys[event.key])
        cid= self.innerfig.canvas.mpl_connect('key_press_event', on_kp_event)
        return cid

class PubSubDisplayTemplate(ABC):
    """
    A template class for creating a display that subscribes to a topic and updates a plot. This class is not meant to be instantiated directly, but rather to be subclassed to create custom displays for different types of data.

    Subclasses must implement the configurePlot and update methods to configure the plot and update it with new data, respectively.

    Attributes:
        topic (str): The topic to subscribe to.
        subplot_kw (dict): Keyword arguments for the subplot.
        pubSubInterface (object): An interface for publishing and subscribing to messages.
    Methods:
        initPlot(fig, gs, idx):
            Initializes a subplot and configures it.
        filterMessages(messages):
            Filters messages to only include those that match the topic.
        getMessageContents(messages):
            Extracts the payload from the filtered messages.
        configurePlot(ax):
            Abstract method to configure the plot. Must be implemented by subclasses.
        update(frame, data, extra):
            Abstract method to update the plot. Must be implemented by subclasses.
    """
    def __init__(self, topic=None, subplot_kw = {}, pubSubInterface = None):
        self.topic = topic
        self.subplot_kw = subplot_kw
        self.pubSubInterface = pubSubInterface
        if not pubSubInterface:
            raise ValueError("PubSubInterface is required")
        
    
    def initPlot(self,fig,gs,idx):
        """
        Initializes a plot within a given figure and grid specification.

        Parameters:
        fig (matplotlib.figure.Figure): The figure object to which the subplot will be added.
        gs (matplotlib.gridspec.GridSpec): The grid specification defining the layout of the subplots.
        idx (int): The index within the grid specification where the subplot will be placed.

        Returns:
        matplotlib.axes._subplots.AxesSubplot: The created subplot axes.
        """
        ax = fig.add_subplot(gs[idx], **self.subplot_kw)
        self.configurePlot(ax)
        return ax
    
    def filterMessages(self, messages):
        """
        Filters a list of messages to include only those that match the instance's topic. This method is included to provide a convenient way to filter incoming pub/sub messages, since the display process must listen to all display messages, but this plot should only update corresponding to its topic.

        Args:
            messages (list): A list of message objects to be filtered.

        Returns:
            list: A list of messages that match the instance's topic.
        """
        return [m for m in messages if self.pubSubInterface.getTopic(m) == self.topic]
    
    def getMessageContents(self, messages):
        """
        Extracts and returns the payload from a list of messages. This method is included to provide a convenient way to extract the payload from a list of messages.

        Args:
            messages (list): A list of message objects.

        Returns:
            list: A list containing the payloads of the provided messages.
        """
        return [self.pubSubInterface.getPayload(m) for m in messages]


    @abstractmethod 
    def configurePlot(self, ax,):
        """
        Configures the plot with the given axes. This method must be implemented by subclasses to configure the plot with the provided axes object. Subclasses will be provided with the axes object to configure the plot as needed.

        Parameters:
        ax (matplotlib.axes.Axes): The axes object to configure.

        Returns:
        None
        """
        pass

    @abstractmethod
    def update(self, frame, data, extra):
        """
        Update the display with the given frame, data, and extra information. This method must be implemented by subclasses to update the display with new data.

        Args:
            frame: The current frame to be displayed.
            data: The data to be used for updating the display.
            extra: Additional information required for the update.
        """
        pass
        
class LidarBasicDisplay(PubSubDisplayTemplate):
    """
    A class to display LIDAR data using a polar plot. This class parses LIDAR scan data and displays it in a polar plot. 
    
    This class includes an argument for a post-processing function to that is applied to the LIDAR data before displaying it. This function can be used to filter or modify the data before plotting it. For example, if the LIDAR data contains a large number of points, it is recommended to use this function to reduce the number of data points displayed on the plot to improve performance.

    By default, this class will look for messages on the 'lidar/scan' topic, which should contain the LIDAR scan data. The scan data is expected to be a tuple of two lists: the angles and distances of the LIDAR scan.

    Attributes:
        title (str): The title of the plot.
        initial_data (tuple): Initial data for the plot in the form of ([angles], [distances]).
        maximumSamplesinRound (int): Maximum number of samples to display in one round.
        max_display_distance (int): Maximum distance to display on the plot.
        post_process (callable): A function to process the data before displaying.
        post_process_kwargs (dict): Keyword arguments for the post_process function.
    Methods:
        configurePlot(ax):
            Configures the polar plot with initial settings and data.
        update(frame, data, extra):
            Updates the plot with new LIDAR data.
    """
    def __init__(self, 
                pubSubInterface = None,
                title='RPLidar A1M8 Full Scan', 
                initial_data = ([0],[0]), 
                maximumSamplesinRound=1000, 
                max_display_distance=2000,
                post_process = None, post_process_kwargs = {}):
        """
        Initializes the AlexDisplay class.
        Parameters:
        pubSubInterface (object, optional): The PubSub interface to use for communication. Defaults to None.
        title (str, optional): The title of the display. Defaults to 'RPLidar A1M8 Full Scan'.
        initial_data (tuple, optional): Initial data to display, in the form of a tuple of two lists. Defaults to ([0], [0]).
        maximumSamplesinRound (int, optional): Maximum number of samples to display in one round. Defaults to 1000.
        max_display_distance (int, optional): Maximum distance to display on the radar. Defaults to 2000.
        post_process (callable, optional): A function to process the data before displaying. Defaults to None.
        post_process_kwargs (dict, optional): Keyword arguments for the post_process function. Defaults to {}.
        """
        PubSubDisplayTemplate.__init__(
            self,topic="lidar/scan",subplot_kw={"projection":"polar"} ,pubSubInterface=pubSubInterface)
        self.title = title
        self.initial_data = initial_data
        self.maximumSamplesinRound = maximumSamplesinRound
        self.max_display_distance = max_display_distance

        self.post_process = post_process
        self.post_process_kwargs = post_process_kwargs

    def configurePlot(self, ax):
        """
        Configures the polar plot for displaying LIDAR data.
        Parameters:
        ax (matplotlib.axes._subplots.PolarAxesSubplot): The polar axes subplot to configure.
        Returns:
        matplotlib.collections.PathCollection: The scatter plot points representing the initial LIDAR data.
        This method sets the direction and zero location for the theta axis, adds a title with extra padding,
        and plots the initial LIDAR data points on the polar plot. It also sets the maximum display distance
        for the radial axis.
        """

        # set the direction and zero location for the theta axis (angle)
        ax.set_theta_direction(-1)
        ax.set_theta_zero_location('N')

        # add more space for the title
        ax.set_title(self.title, pad=20)
        initial_angle = np.array(self.initial_data[0][:self.maximumSamplesinRound])/180 * np.pi
        initial_distance = self.initial_data[1][:self.maximumSamplesinRound]

        points=ax.scatter(initial_angle, initial_distance, marker='o', s=2)
        ax.set_rmax(self.max_display_distance)

        self.points = points
        return points

    def update(self, frame, data, extra):
        """
        Update the display with new LIDAR data.
        Parameters:
        frame (int): The current frame number, in terms of matplotlib animation frames.
        data (list): The raw data received from the LIDAR sensor. This data is expected to be pubsub messages, which are filtered and processed to extract the relevant scan data.
        extra (dict): Additional information that might be needed for processing.
        Returns:
        tuple: A tuple containing the updated points to be displayed.
        This method processes the incoming LIDAR data, filters the relevant messages,
        extracts the scan data, and updates the display points accordingly. If a 
        post-processing function is provided, it applies the function to the angle 
        and distance data before updating the display.
        """
        # Get the appropriate messages
        messages = self.filterMessages(data)
        if not messages:
            return (self.points,)
        
        # Get the contents of the messages
        scans = self.getMessageContents(messages)

        # Only use the most recent scan, and discard the rest
        # Limit the number of samples to the maximum in a round
        scan = scans[-1]
        angleData, distanceData, qualityData = scan
        angleData = scan[0][:self.maximumSamplesinRound]
        distanceData = scan[1][:self.maximumSamplesinRound]


        dist,angle = distanceData,angleData
        if not (self.post_process is None):
            dist,angle = self.post_process(distanceData,angleData, **self.post_process_kwargs)

        angle = (np.array(angle))/180 * np.pi
        dist = np.array(dist)

        self.points.set_offsets(np.stack([angle, dist]).T)
        return (self.points,)

class LidarSlamDisplay(PubSubDisplayTemplate):
    """
    LidarSlamDisplay is a class for visualizing SLAM (Simultaneous Localization and Mapping) data from a LIDAR sensor. 
    This class is built to display SLAM data from BreezySLAM, which provides a byte array representing the map, as well as the vehicle pose (x, y, theta) in millimeters and degrees.

    By default, this class will look for messages on the 'slam/mappose' topic, which should contain the SLAM data. The map data is expected to be a byte array, while the vehicle pose is expected to be a tuple of (x, y, theta) in millimeters and degrees.

    Attributes:
        ROBOT_HEIGHT_M (float): Height of the robot in meters.
        ROBOT_WIDTH_M (float): Width of the robot in meters.
        map_size_pixels (int): Size of the map in pixels.
        map_scale_meters_per_pixel (float): Scale of the map in meters per pixel.
        prevpos (tuple): Previous position of the robot for trajectory visualization.
        showtraj (bool): Flag to show the trajectory of the robot.
        zero_angle (float): Initial angle for rotation correction.
        start_angle (float): Starting angle for rotation correction.
        rotate_angle (float): Angle to rotate the map for correction.
        title (str): Title of the plot.
        shift (int): Shift value for centering the map.
    Methods:
        __init__(pubSubInterface=None, title='RPLidar A1M8 SLAM', map_size_pixels=1000, map_size_meters=10, shift=0, show_trajectory=False, zero_angle=0):
            Initializes the LidarSlamDisplay with given parameters.
        configurePlot(ax):
            Configures the plot with axes labels, grid, and initial map setup.
        update(frame, data, extra):
            Updates the display with new SLAM data.
        _updateMap(mapbytes):
            Updates the map with new data from a byte array.
        _setPose(x_m, y_m, theta_deg):
            Sets the vehicle pose on the map.
        _m2pix(x_m, y_m):
            Converts meters to pixels for map scaling.
    """
    ROBOT_HEIGHT_M = 0.25
    ROBOT_WIDTH_M  = 0.15

    def __init__(self, 
                pubSubInterface = None,
                title='RPLidar A1M8 SLAM', 
                map_size_pixels = 1000, map_size_meters = 10,
                shift = 0,
                show_trajectory = False,zero_angle=0,
                ):
        
        # Initialize the PubSubDisplayTemplate
        PubSubDisplayTemplate.__init__(
            self,topic="slam/mappose",subplot_kw={} ,pubSubInterface=pubSubInterface)

        
        # Store constants for update
        map_size_meters = map_size_meters
        self.map_size_pixels = map_size_pixels
        self.map_scale_meters_per_pixel = map_size_meters / float(map_size_pixels)

        # Store previous position for trajectory
        self.prevpos = None
        self.showtraj = show_trajectory

        # Handle angles 
        self.zero_angle = zero_angle
        self.start_angle = None
        self.rotate_angle = 0

        # Store the title
        self.title = title

        # shift for centering the map
        self.shift = shift

    def configurePlot(self, ax):
        """
        Configures the plot for displaying LIDAR data.
        Parameters:
        ax (matplotlib.axes.Axes): The axes object to configure.
        Returns:
        tuple: A tuple containing the image artist and the vehicle artist.
        This method sets up the plot with appropriate labels, ticks, and limits.
        It also initializes a dummy image and a vehicle arrow for visualization.
        """


        # add more space for the title
        ax.set_title(self.title, pad=20)

        # Set up shift 
        shift = self.shift

        # Create axes
        self.ax:axes.Axes = ax
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.grid(False)

        # Hence we must relabel the axis ticks to show millimeters
        ticks = np.arange(shift,self.map_size_pixels+shift+100,100)
        labels = [str(self.map_scale_meters_per_pixel * tick) for tick in ticks]
        self.ax.set_xticks(ticks)
        self.ax.set_yticks(ticks)
        self.ax.set_xticklabels(labels)
        self.ax.set_yticklabels(labels)

        # We base the axis on pixels, to support displaying the map
        self.ax.set_xlim([shift, self.map_size_pixels+shift])
        self.ax.set_ylim([shift, self.map_size_pixels+shift])

        # Set up default shift for centering at origin
        shift = -self.map_size_pixels / 2

        # Add a dummy image for initial display
        dummy = np.full((self.map_size_pixels, self.map_size_pixels), 125, dtype=np.uint8)
        self.img_artist = self.ax.imshow(dummy, cmap=colormap.gray, origin = 'lower', animated=True, vmin=0, vmax=255)

        # invert y axis to match the map. Breezy SLAM uses a diferent origin and coordinate system compared to Matplotlib's imshow 
        # So we need to invert the y axis to match the Breezy SLAM map with the Matplotlib imshow
        self.ax.invert_yaxis()

        # vehicle artist
        self.vehicle = self.ax.arrow(0,0,0,0, head_width=0.1, head_length=0.2, fc='r', ec='r', animated=True)

        return self.img_artist, self.vehicle

    def update(self, frame, data, extra):
        """
        Update the display with the latest frame and data.
        Parameters:
        frame (int): The current frame number.
        data (list): A list of data messages to be processed.
        extra (dict): Additional information that might be needed for the update.
        Returns:
        tuple: A tuple containing the updated image artist and vehicle pose.
        """
        # Get the appropriate messages
        messages = self.filterMessages(data)
        if not messages:
            return (self.img_artist, self.vehicle)
        
        # Get the contents of the messages
        scans = self.getMessageContents(messages)

        # Most recent slam maps
        scan = scans[-1]
        x, y, theta, mapbytes = scan

        # Update the map
        self._updateMap(mapbytes)

        # Update the vehicle pose
        self._setPose(x/1000., y/1000., theta)

        # print(f"X: {x} | Y: {y} | Theta: {theta}")

        return (self.img_artist, self.vehicle)



    def _updateMap(self, mapbytes):
        '''
        Updates the map with new data. BreezySLAM provides the map as a byte array, we need to convert it to a 2D numpy array to update the display. 
        '''
        # Convert the byte array to a numpy array
        mapimg = np.reshape(np.frombuffer(mapbytes, dtype=np.uint8), (self.map_size_pixels, self.map_size_pixels))

        # Update the image
        self.img_artist.set_data(mapimg)

    def _setPose(self, x_m, y_m, theta_deg):
        '''
        Sets vehicle pose:
        X:      left/right   (m)
        Y:      forward/back (m)
        theta:  rotation (degrees)
        '''

        # If zero-angle was indicated, grab first angle to compute rotation
        if self.start_angle is None and self.zero_angle != 0: 
            self.start_angle = theta_deg
            self.rotate_angle = self.zero_angle - self.start_angle

        # Rotate by computed angle, or zero if no zero-angle indicated (this is for when the entire map is rotated)
        # d is the angle to rotate by
        # a is the angle to rotate in radians
        # c is the cosine of the angle
        # s is the sine of the angle
        # x_m and y_m are the x and y coordinates in meters
        d = self.rotate_angle
        a = np.radians(d)
        c = np.cos(a)
        s = np.sin(a)
        x_m , y_m = x_m*c - y_m*s, y_m*c + x_m*s

        # Use a very short arrow shaft to orient the head of the arrow
        # We negate the theta to match the map orientation wrt to the difference in coordinate systems of BreezySLAM and Matplotlib
        theta_rad = np.radians(-theta_deg+d)
        c = np.cos(theta_rad)
        s = np.sin(theta_rad)
        l = 0.1
        dx = l * c
        dy = l * s
 
        s = self.map_scale_meters_per_pixel

        self.vehicle.set_data(x=x_m/s, y=y_m/s, dx=dx, dy=-dy, head_width=self.ROBOT_WIDTH_M/s, head_length=self.ROBOT_HEIGHT_M/s)

        # Show trajectory if indicated
        # currpos = self._m2pix(x_m,y_m)
        # if self.showtraj and not self.prevpos is None:
        #     self.ax.add_line(mlines.Line2D((self.prevpos[0],currpos[0]), (self.prevpos[1],currpos[1])))
        # self.prevpos = currpos

    def _m2pix(self, x_m, y_m):
        """
        Convert coordinates from meters to pixels based on the map scale.
        Args:
            x_m (float): The x-coordinate in meters.
            y_m (float): The y-coordinate in meters.
        Returns:
            tuple: A tuple containing the x and y coordinates in pixels.
        """

        s = self.map_scale_meters_per_pixel

        return x_m/s, y_m/s




# Functional Version
_PLOT_DATA_BUFFER = []

def simple_scatter_plot(
    Xs, Ys,
    title="LiDAR single scan",
    xlabel="X Coordinates (mm)",
    ylabel="Y Coordinates (mm)",
    
    # Scatter plot keyword arguments
    plot_keyword_arguments = {
        's': 1,  # size of the points
        'c': 'blue',  # color of the points
        'alpha': 1  # transparency of the points
    }

    ):
    
    # Plot the scan data using matplotlib. Matplotlib is a powerful plotting library for Python, and is widely used in the scientific community. Examples of its use can be found at https://matplotlib.org/stable/gallery/index.html

    # we will be plotting a very simple scatter plot of the scan data
    # we will create a figure and an axis object to plot on
    # subplots() is a function that creates a figure and rows x cols grid of subplots, returning a figure and an array of axes objects (unless rows and cols are both 1, in which case it returns a single axes object)
    figure, ax = plt.subplots(nrows=1, ncols=1, figsize=(10, 10))

    # scatter() is a function that creates a scatter plot of x and y data
    # we will plot the scan data as a scatter plot. However, the data we have is in polar coordinates (angle, distance), and scatter() expects cartesian coordinates (x, y).
    # We will therefore convert the polar coordinates to cartesian coordinates.



    # Plotting the scan data
    # scatter() is a function that creates a scatter plot of x and y data
    # we will plot the scan data as a scatter plot. 
    #  **plot_keyword_arguments is a way to pass a dictionary of keyword arguments to a function. scatter() takes a number of keyword arguments, and we are passing them in this way.
    # this is the same as calling scatter(Xs, Ys, s=1, c='blue', alpha=0.5)
    ax.scatter(Xs, Ys, **plot_keyword_arguments)

    # Now we label the axes
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)

    # make sure that the aspect ratio of the plot is equal, so that the distances are represented accurately
    ax.set_aspect('equal', 'box')

    # set the title of the plot
    ax.set_title(title)

    # show the plot
    plt.show()


def simple_animated_scatter_plot():
    """
    Main function to demonstrate a simple animated scatter plot using matplotlib.
    """
    # Create a figure and axis object
    fig, ax = plt.subplots()
    # Create an empty scatter plot
    sc = ax.scatter([], [])
    # Set the axis limits
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)

    def update(frame):
        """
        Update the scatter plot with new data.
        """
        # Generate random data for the scatter plot
        x = np.random.rand(100)
        y = np.random.rand(100)
        # Update the scatter plot with the new data
        sc.set_offsets(np.column_stack((x, y)))
        return sc,

    # Create an animation of the scatter plot
    ani = animation.FuncAnimation(fig, update, frames=100, interval=100)
    # Show the animation
    plt.show()


