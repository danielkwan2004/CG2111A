# This node is similar to the cli_node in that it publishes messages to the arduino/send topic. Instead of taking user input locally however, it takes input from a remote TLS client that connects to a local TLS server. This node should only be run as a thread unless you know what you are doing.

# Import Python Native Modules. We require the Barrier class from threading to synchronize the start of multiple threads.
from threading import Barrier
import os, struct,time

# Import the required pubsub modules. PubSubMsg class to extract the payload from a message.
from pubsub.pub_sub_manager import ManagedPubSubRunnable, PubSubMsg
from pubsub.pub_sub_manager import publish, subscribe, unsubscribe, getMessages, getCurrentExecutionContext  

# Import the required arduino communication modules. Replace or add to the handlers as needed.
from control.alex_control import parseUserInput

# Import the required Network communication modules.
from networking.sslServer import setupTLSServer, acceptTLSConnection, isServerAlive, shutdownServer
from networking.sslServer import sendNetworkData, recvNetworkData, isTLSConnected, disconnect, getPeerDetails
from networking.constants import TNetType

# Constants
# file location 
keyFileLocations = os.path.join(os.path.dirname(os.path.abspath(__file__)), "TLS_node_data")

host = "0.0.0.0" # listen on all interfaces
port = 5000 # listen on port 5000
serverKeyPath = os.path.join(keyFileLocations, "alex.key") # Replace with the server key path
serverCertPath = os.path.join(keyFileLocations, "alex.crt") # Replace with the server certificate path
caCertPath = os.path.join(keyFileLocations, "signing.pem") # Replace with the CA certificate path
expectedClientName = "laptop.epp.com" # Replace with the expected client name

# Buffer and message Properties
NET_TYPE_SIZE = struct.calcsize("=B") #standard unsigned C char, maps to a python int
CMD_SIZE = struct.calcsize("=c") #standard unsigned C char, maps to a python int
PARAM_SIZE = struct.calcsize("=I") #standard unsigned C int_32, maps to a python int
EXPECTED_MESSAGE_SIZE = NET_TYPE_SIZE*1 + CMD_SIZE*1 + PARAM_SIZE*2
BUFFERSIZE = 10
assert EXPECTED_MESSAGE_SIZE <= BUFFERSIZE, "Buffer size is too small for expected message size."

# Topics
ARDUINO_SEND_TOPIC = "arduino/send"



def TLSRecvThread(setupBarrier:Barrier=None, readyBarrier:Barrier=None):
    """
    Thread function to handle receiving data over a TLS connection in a loop until the context signals an exit.
    Args:
        readyBarrier (Barrier, optional): A threading barrier to synchronize the start of the thread.
                                           If provided, the thread will wait for all parties to be ready before proceeding.
    The function performs the following steps:
    1. Sets up the execution context.
    2. Waits for all threads to be ready if a barrier is provided.
    3. Sets up the TLS server to accept incoming connections.
    4. Enters a loop to receive data over the TLS connection until the context signals an exit.
    5. Processes received messages and dispatches them accordingly.
    6. Gracefully shuts down the TLS server and disconnects the connection.
    """
    # Setup
    ctx:ManagedPubSubRunnable = getCurrentExecutionContext()

    # Perform any setup here
    setupBarrier.wait() if readyBarrier != None else None

    serverRunning = setupTLSServer(host, port, serverKeyPath, serverCertPath, caCertPath, expectedClientName)

    # If server failed to run, trigger an early exit
    # Else print the server information
    if not serverRunning:
        print("Failed to start the TLS server. Will exit when setup process completes.")
        ctx.doExit() 
    else:
        # get the address and port from the socket
        serverInfo = "Server Started!\n"
        serverInfo = serverInfo + f"Server Address: {host}\n"
        serverInfo = serverInfo + f"Server Port: {port}\n"
        serverInfo = serverInfo + f"Server Key Path: {serverKeyPath}\n"
        serverInfo = serverInfo + f"Server Cert Path: {serverCertPath}\n"
        serverInfo = serverInfo + f"CA Cert Path: {caCertPath}\n"
        serverInfo = serverInfo + f"Client FQDN: {expectedClientName}\n"
        print(serverInfo)

    # Wait for all Threads ready
    readyBarrier.wait() if readyBarrier != None else None

    # check if the server is running and there is a connection
    try:
        while (not ctx.isExit()) and serverRunning:
            acceptResult = acceptTLSConnection(timeout=1)

            if acceptResult:
                clientIP, ClientPort = getPeerDetails()
                print(f"Connection Accepted from: {clientIP}:{ClientPort}")
                connectionBuffer = bytearray(BUFFERSIZE)
                offset = 0

                while (not ctx.isExit()) and isTLSConnected():
                    networkMessage, size = recvNetworkData(1024)
                    if size > 0:
                        # put into buffer
                        for i in range(size):
                            connectionBuffer[offset] = networkMessage[i]
                            offset += 1
                            if offset == EXPECTED_MESSAGE_SIZE:
                                # handle message
                                handleNetworkData(connectionBuffer[0:EXPECTED_MESSAGE_SIZE])
                                offset = 0
                    elif size == 0:
                    # No message received
                        continue
                    else:
                    # Error occurred
                        serverRunning = isServerAlive()
                        print("Connection Error. Disconnecting...")
                        disconnect()
                        break
            else:
                time.sleep(1) 
    
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        pass
    

    # Shutdown and exit the thread gracefully
    print("Shutting down TLS Server")
    disconnect()
    shutdownServer()
    ctx.doExit()
    print("Exiting TLS Recv Thread")
    

    
def handleNetworkData(buffer:bytes):
    """
    Handles the received message from the TLS connection.
    Args:
        message (str): The message received from the TLS connection.
    """
    # First we check what kind of network message we received
    packetType = struct.unpack("=B", buffer[0:CMD_SIZE])[0] 

    if packetType == TNetType.NET_COMMAND_PACKET.value:
        # Handle Command
        handleCommand(buffer[NET_TYPE_SIZE:])
        pass

    else:
        # Only handling command packets is implemented, you can add other types
        pass
    
    
def handleCommand(buffer:bytes):
    """
    Handles the received command from the TLS connection. Expects the the following format:
    [Command (1 byte)][Param1 (4 bytes)][Param2 (4 bytes)]
    Args:
        message (str): The message received from the TLS connection.
    """
    command = struct.unpack("=c", buffer[0:CMD_SIZE])[0].decode("utf-8")
    param1 = struct.unpack("=I", buffer[CMD_SIZE:CMD_SIZE+PARAM_SIZE])[0]
    param2 = struct.unpack("=I", buffer[CMD_SIZE+PARAM_SIZE:CMD_SIZE+2*PARAM_SIZE])[0]

    # Publish the command to the arduino/send topic
    # (packetType, commandType,  params)
    input_str = f"{command} {param1} {param2}"
    parseResult = parseUserInput(input_str, exitFlag=getCurrentExecutionContext().exitEvent)
    # if the parse result is None then the node received an invalid command
    if parseResult == None:
        # print("Invalid command. Please try again.")
        return False
    else:
        # if the parse result is not None then the user entered a valid command
        # and the command has been published to the "arduino/send" topic
        return publish(ARDUINO_SEND_TOPIC, tuple(parseResult))

       
    
    