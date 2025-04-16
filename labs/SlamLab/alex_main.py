"""
This module provides a multi-threaded and multi-process system for controlling an Arduino, scanning with a Lidar sensor, 
performing SLAM (Simultaneous Localization and Mapping), managing TLS network relays, and providing a user interface.

Modules and Functions:
- control.alex_control_serial: Functions for managing serial communication with the Arduino (startSerial, closeSerial).
- control.alex_control: Functions for handling the system's communication routines (e.g., waitForHelloRoutine).
- pubsub.pub_sub_manager: Manager for inter-thread and inter-process publish/subscribe communication.
- nodes.alex_arduino_send_node: Contains sendThread for sending packets to the Arduino.
- nodes.alex_arduino_receive_node: Contains receiveThread for receiving packets from the Arduino.
- nodes.alex_lidar_scan_node: Contains lidarScanThread for Lidar sensor scanning.
- nodes.alex_cli_node: Contains cliThread for handling CLI input.
- nodes.alex_slam_node: Contains slamThread for SLAM processing.
- nodes.alex_display_node: Contains lidarDisplayProcess for displaying Lidar scan and SLAM data.
- nodes.alex_TLSRelay_recv_node: Contains TLSRecvThread for receiving data from the TLS relay.
- nodes.alex_TLSRelay_send_node: Contains TLSSendThread for sending data through the TLS relay.
- nodes.alex_message_monitor_node: Contains monitorThread for debugging and message monitoring.

Global Constants:
- VERBOSE: Flag for enabling verbose output.
- PORT_NAME, BAUD_RATE, BYTE_SIZE, PARITY, STOP_BITS: Parameters for Arduino serial communication.
- MAX_ATTEMPTS, SERIAL_TIMEOUT, FAILED_ATTEMPT_WAIT_SEC: Parameters controlling serial connection attempts.
- ENABLE_DEBUG_MESSAGE_MONITOR, ENABLE_LIDAR_NODE, ENABLE_SLAM_NODE, ENABLE_GUI_NODE, ENABLE_CLI_NODE, ENABLE_TLS_SERVER, ENABLE_ARDUNIO_INTERFACE:
    Flags to enable the corresponding system components.

Threads:
- Arduino Threads: sendThread and receiveThread for Arduino communication.
- Lidar Thread: lidarScanThread for Lidar sensor scanning.
- SLAM Thread: slamThread for SLAM processing.
- TLS Relay Threads: TLSRecvThread and TLSSendThread for handling TLS network communications (active when ENABLE_TLS_SERVER is True).
- UI Thread: cliThread for the command-line interface.
- Debug Monitor Thread: monitorThread for outputting debug messages.

Processes:
- UI Process: lidarDisplayProcess for graphical display of Lidar scan and SLAM data.

Main Function:
- main(): Initializes serial communication (if enabled), synchronizes threads and processes using barriers, sets up each component (Arduino, Lidar, SLAM, TLS Relay, UI, and Debug Monitor), handles the Arduino handshake, and gracefully starts and joins all threads and processes.
"""

import time,sys
from threading import Barrier
from multiprocessing import Barrier as mBarrier

from control.alex_control_serial import startSerial, closeSerial
from control.alex_control import  waitForHelloRoutine
from pubsub.pub_sub_manager import PubSubManager


###########################
######## Constants ########
###########################
# General
VERBOSE = True

# Arduino Serial Constants
PORT_NAME = "/dev/ttyACM0"
BAUD_RATE = 9600
BYTE_SIZE = 8
PARITY = "N"
STOP_BITS = 1
MAX_ATTEMPTS = 5
SERIAL_TIMEOUT = 1
FAILED_ATTEMPT_WAIT_SEC = 5


# NODE FLAGS
ENABLE_DEBUG_MESSAGE_MONITOR = False 
ENABLE_LIDAR_NODE = True
ENABLE_SLAM_NODE = True
ENABLE_GUI_NODE = True
ENABLE_CLI_NODE = False
ENABLE_TLS_SERVER = False
ENABLE_ARDUNIO_INTERFACE = False

DEBUG_MONITOR_TOPICS = [""]



########################
######## NODES #########
########################

# ARDUINO INTERFACE
from nodes.alex_arduino_receive_node import receiveThread
from nodes.alex_arduino_send_node import sendThread

# CLI COMMAND NODE
from nodes.alex_cli_node import cliThread

# LIDAR and SLAM NODES
from nodes.alex_lidar_scan_node import lidarScanThread
from nodes.alex_slam_node import slamThread

# LIDAR DISPLAY NODE
#from nodes.alex_display_node import lidarDisplayProcess

# Alternative display backend! Try it out if you dare
# Requires Bokeh. pip install bokeh
from nodes.alex_bokeh_display_node import lidarDisplayProcess	###

# TLS RELAY NODES
from nodes.alex_TLSRelay_recv_node import TLSRecvThread
from nodes.alex_TLSRelay_send_node import TLSSendThread

# DEBUG MESSAGE MONITOR NODE
from nodes.alex_message_monitor_node import monitorThread


def main():
    """
    Main function to set up and manage the communication and processes for the system.
    This function performs the following steps:
    1. Initializes the serial communication with specified parameters.
    2. Waits for the Arduino to reset.
    3. Establishes thread and process barriers for synchronization.
    4. Adds and starts various threads and processes for communication, LIDAR scanning, SLAM, and UI.
    5. Performs a handshake with the Arduino.
    6. Waits for all threads and processes to be ready.
    7. Waits for threads to finish and then cleans up and exits gracefully.
    Returns:
        None
    """

    if ENABLE_ARDUNIO_INTERFACE:
        print("==============SETTING UP==============")

        res = startSerial(
            portName=PORT_NAME, baudRate=BAUD_RATE, byteSize=BYTE_SIZE, parity=PARITY, stopBits=STOP_BITS, maxAttempts=MAX_ATTEMPTS,
            serialTimeout=SERIAL_TIMEOUT, failedAttemptWaitSec=FAILED_ATTEMPT_WAIT_SEC
        )
        if not res:
            print ("Failed to open serial port. Exiting...")
            return
        print("Serial OK")
        
        # Wait for arduino to reset
        print("WAITING TWO SECONDS FOR ARDUINO TO REBOOT.", end="", flush=True)
        # make sure to flush the print buffer before sleeping
        time.sleep(2/3)
        print(".", end="",  flush=True)
        time.sleep(2/3)
        print(".", end="", flush=True)
        time.sleep(2/3)
        print("DONE",  flush=True)

    # Establish Threads Signatures
    with PubSubManager() as mgr:
        # Establishing Barriers for Threads and Processes
        ## Barriers are flow control primitives that allow multiple threads/processes to wait until all parties have reached a certain point of execution before any can proceed. 
        # This is important for synchronization and ensuring that all components are ready before starting the main workflow.
        ## Barrier(2) -> 2 parties (i.e., main thread and 1 sub-thread) must call wait() before any can proceed
        # The barrier will only allow all waiting parties to proceed once the specified number of parties have called wait()

        # In general, the way we use barriers is as follows:
        # 1. Create a barrier with the number of threads/processes + 1 (for the main thread)
        # 2. Each thread/process calls setupBarrier.wait() to indicate that it is ready to proceed with setup
        # 3. The main thread will call setupBarrier.wait() to "allow" the threads/processes to proceed with setup
        # 4. Each thread/process will call readyBarrier.wait() to indicate that it is ready to proceed with the main workflow
        # 5. The main thread will call readyBarrier.wait() to "allow" the threads/processes to proceed with the main workflow
        # This way we can use the main thread to control the order of execution for the setup and main workflow of the threads/processes


        if ENABLE_ARDUNIO_INTERFACE:
            # Arduino Threads Syncronisation
            arduinoNodes = 2 # 2 threads for arduino: send and receive
            setupBarrier_arduino_t = Barrier(arduinoNodes + 1) # 2 threads + 1 main thread.
            readyBarrier_arduino_t = Barrier(arduinoNodes + 1) # 2 threads + 1 main thread.

        if ENABLE_LIDAR_NODE:
            # Lidar Threads Syncronisation
            lidarNodes = 1 # 1 threads for lidar: scan
            setupBarrier_lidar_t = Barrier(lidarNodes + 1) # 2 threads + 1 main thread.
            readyBarrier_lidar_t = Barrier(lidarNodes + 1) # 2 threads + 1 main thread.

        if ENABLE_SLAM_NODE:
            # SLAM Threads Syncronisation
            slamNodes = 1 # 1 thread for slam: slamThread
            setupBarrier_slam_t = Barrier(slamNodes + 1) # 1 thread + 1 main thread.
            readyBarrier_slam_t = Barrier(slamNodes + 1) # 1 thread + 1 main thread.

        if ENABLE_TLS_SERVER:
            # TLS Relay Threads Syncronisation
            networkTLSNodes = 2 # 2 threads for TLS Relay: TLSRecvThread and TLSSendThread
            setupBarrierNetworkTLS_t = Barrier(networkTLSNodes + 1) # 2 threads + 1 main thread.
            readyBarrierNetworkTLS_t = Barrier(networkTLSNodes + 1) # 2 threads + 1 main thread.

        if ENABLE_CLI_NODE:
            # UI Threads Syncronisation
            uiThreads = 1 # 1 thread for UI: cli (Thread)
            setupBarrier_ui_t = Barrier(uiThreads + 1) # 1 thread + 1 main thread.
            readyBarrier_ui_t = Barrier(uiThreads + 1) # 1 thread + 1 main thread.

        if ENABLE_GUI_NODE:
            # UI Process Syncronisation
            uiProcesses = 1 # 1 process for UI: lidarDisplay (Process)
            setupBarrier_ui_m = mBarrier(uiProcesses + 1) # 1 process + 1 main process
            readyBarrier_ui_m = mBarrier(uiProcesses + 1) # 1 sub_process + 1 parent process

        if ENABLE_DEBUG_MESSAGE_MONITOR:
            # Debug Thread Synronisation
            monitorNodes = 1 # 1 thread for monitoring messages
            setupBarrier_monitor_t = Barrier(monitorNodes + 1) # 1 thread + 1 main thread.
            readyBarrier_monitor_t = Barrier(monitorNodes + 1) # 1 thread + 1 main thread.


        if ENABLE_DEBUG_MESSAGE_MONITOR:
            # Adding Monitor Thread
            mgr.add_thread(target=monitorThread,
                    name="Monitor Thread",
                    kwargs={"setupBarrier": setupBarrier_monitor_t, 
                            "readyBarrier": readyBarrier_monitor_t, 
                            "topicsToMonitor": DEBUG_MONITOR_TOPICS})


        if ENABLE_ARDUNIO_INTERFACE:
            # Adding Arduino Threads
            mgr.add_thread(target=sendThread, 
                    name="Arduino Send Thread",
                    kwargs={"setupBarrier": setupBarrier_arduino_t, "readyBarrier": readyBarrier_arduino_t})
            mgr.add_thread(target=receiveThread, 
                    name="Arduino Receive Thread",
                    kwargs={"setupBarrier": setupBarrier_arduino_t, "readyBarrier": readyBarrier_arduino_t})
            
        if ENABLE_LIDAR_NODE:
            # Adding Lidar Thread
            mgr.add_thread(target=lidarScanThread, 
                    name="Lidar Scan Thread",
                    kwargs={"setupBarrier": setupBarrier_lidar_t, "readyBarrier": readyBarrier_lidar_t})
            
        if ENABLE_SLAM_NODE:
            # Adding SLAM Thread
            mgr.add_thread(target=slamThread, 
                    name="SLAM Thread",
                    kwargs={"setupBarrier": setupBarrier_slam_t, "readyBarrier": readyBarrier_slam_t})
        
        if ENABLE_CLI_NODE:
            # Adding CLI Threads
            mgr.add_thread(target=cliThread, 
                    name="CLI Thread",
                    kwargs={"setupBarrier": setupBarrier_ui_t, "readyBarrier": readyBarrier_ui_t})
        
        if ENABLE_GUI_NODE:
            # Adding GUI Process
            mgr.add_process(target=lidarDisplayProcess, 
                    name="Lidar Display Process",
                    kwargs={"setupBarrier": setupBarrier_ui_m, "readyBarrier": readyBarrier_ui_m})

        # Adding TLS Relay Threads
        if ENABLE_TLS_SERVER:
            mgr.add_thread(target=TLSRecvThread, 
                   name="TLS Relay Receive Thread",
                   kwargs={"setupBarrier": setupBarrierNetworkTLS_t, "readyBarrier": readyBarrierNetworkTLS_t})
            mgr.add_thread(target=TLSSendThread, 
                   name="TLS Relay Send Thread",
                   kwargs={"setupBarrier": setupBarrierNetworkTLS_t, "readyBarrier": readyBarrierNetworkTLS_t})
            



        # Start all threads. 
        # It is important that your threads do not print anything until all processes have been spawned (due to contention for stdout)
        mgr.start_all()


        if ENABLE_DEBUG_MESSAGE_MONITOR:
            print("\n============ DEBUG MESSAGE MONITOR ============")
            setupBarrier_monitor_t.wait()
            readyBarrier_monitor_t.wait()

        if ENABLE_ARDUNIO_INTERFACE:
            print("\n============ ARDUINO HANDSHAKE ============")
            # Sending hello to arduino
            print("Establishing Connection with Arduino. Sending Hello...")
            
            # wait for hello responses 
            waitForHelloRoutine() # this routine does not use the send thread, instead it sends directly to the serial port
            
            print('\n============ ARDUINO SETUP ============')
            setupBarrier_arduino_t.wait()
            readyBarrier_arduino_t.wait()

        if ENABLE_LIDAR_NODE:
            print("\n============ LIDAR SETUP ============")
            # We wait for the lidar to be ready before proceeding
            setupBarrier_lidar_t.wait()
            readyBarrier_lidar_t.wait()

        if ENABLE_SLAM_NODE:
            print("\n============ SLAM SETUP ============")
            # We wait for the SLAM thread to be ready before proceeding
            setupBarrier_slam_t.wait()
            readyBarrier_slam_t.wait()

        if ENABLE_TLS_SERVER:
            print("\n============ NETWORK TLS SETUP ============")
            setupBarrierNetworkTLS_t.wait()
            readyBarrierNetworkTLS_t.wait()


        if ENABLE_GUI_NODE:
            print("\n============ GUI SETUP ============")
            setupBarrier_ui_m.wait()
            readyBarrier_ui_m.wait()

        if ENABLE_CLI_NODE:
            print("\n============ CLI SETUP ============")
            setupBarrier_ui_t.wait()
            readyBarrier_ui_t.wait()


        #Wait for threads to finish, then clean up and exit gracefully
        mgr.join_all()
        closeSerial()


if __name__ == "__main__":
    main()
    print("Done!")
    sys.exit(0)
