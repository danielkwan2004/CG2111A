# This node is an example of a simple publisher that receives messages from the arduino and publish it on the ardunio/recv topic
# Currently does not do anything else with the messages.

# Import Python Native Modules. We require the Barrier class from threading to synchronize the start of multiple threads.
from threading import Barrier

# Import the required pubsub modules. PubSubMsg class to extract the payload from a message.
from pubsub.pub_sub_manager import ManagedPubSubRunnable, PubSubMsg
from pubsub.pub_sub_manager import publish, subscribe, unsubscribe, getMessages, getCurrentExecutionContext  

# Import the required arduino communication modules. Replace or add to the handlers as needed.
from control.alex_control import receivePacket
from control.alex_control_constants import  TPacket, TPacketType, PAYLOAD_PARAMS_COUNT, PAYLOAD_PACKET_SIZE
from control.alex_control_constants import  TResponseType, TResultType


# Constants
PUBLISH_PACKETS = True
ARDUINO_RECV_TOPIC = "arduino/recv" 



def receiveThread(setupBarrier:Barrier=None, readyBarrier:Barrier=None):
    """
    Thread function to handle receiving arduino packets in a loop until the context signals an exit.
    Args:
        setupBarrier (Barrier, optional): A threading barrier to synchronize the start of the thread setup.
        readyBarrier (Barrier, optional): A threading barrier to synchronize the thread start.
                                          If provided, the thread will wait for all parties to be ready before proceeding.
    The function performs the following steps:
    1. Sets up the execution context.
    2. Waits for all threads to be ready if barriers are provided.
    3. Enters a loop to receive arduino packets until the context signals an exit.
    4. Processes packets based on their type:
        - Handles response packets.
        - Handles error response packets.
        - Handles message packets.
        - Logs unknown packet types.
    5. Gracefully shuts down and exits the thread.
    """
    # Setup
    ctx:ManagedPubSubRunnable = getCurrentExecutionContext()

    # Perform any setup here
    setupBarrier.wait() if readyBarrier != None else None
    
    # Nothing to do there
    print(f"Arduino Receive Thread Ready. Publish to {ARDUINO_RECV_TOPIC}? --> {PUBLISH_PACKETS}")   

    # Wait for all Threads ready
    readyBarrier.wait() if readyBarrier != None else None

    # Receiving Logic Loop
    try:
        while(not ctx.isExit()):
            packet = receivePacket(exitFlag=ctx.exitEvent)
            if packet == None:
                # Continue if no packet received
                continue

            # Handle the packet based on its type
            # Default handlers are provided in the control module
            # Pleasee modify or replace to fit the application requirements
            packetType = TPacketType(packet.packetType)
            
            if packetType == TPacketType.PACKET_TYPE_RESPONSE:
                handleResponse(packet, publishPackets = PUBLISH_PACKETS)
            elif packetType == TPacketType.PACKET_TYPE_ERROR:
                handleErrorResponse(packet, publishPackets = PUBLISH_PACKETS)
            elif packetType == TPacketType.PACKET_TYPE_MESSAGE:
                handleMessage(packet, publishPackets = PUBLISH_PACKETS)
            else:
                print(f"Unknown Packet Type {packetType}")
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Receive Thread Exception: {e}")
        pass

    # Shutdown and exit the thread gracefully
    ctx.doExit()
    print("Exiting Receive Thread")
    pass

##########################
##### PACKET HANDLERS ####
##########################
def handleResponse(res: TPacket, publishPackets:bool=False):
    """
    Handles the response from the Arduino.

    Args:
        res (TPacket): The response packet received from the Arduino.

    Returns:
        Tuple[TResponseType, Tuple[ctypes.c_uint32]]: A tuple containing the response type and parameters

    Prints:
        - "Command OK" if the response type is RESP_OK.
        - "Status OK" if the response type is RESP_STATUS.
        - "Arduino sent unknown response type {res_type}" for any other response type.
    """
    res_type = TResponseType(res.command)
    if res_type == TResponseType.RESP_OK:
        print("Command OK")

        if publishPackets:
            publish("arduino/recv", (res.packetType, res.command ))

    elif res_type == TResponseType.RESP_STATUS:
        # we assume that the status if stored in the parameters
        # we will print the status
        params = tuple([p for p in res.params])
        status_str = ""
        for idx, p in enumerate(params):
            # We don't know what your parameters are, so we will just print them as is
            # You can modify this to fit your application
            param_name = "param" + str(idx)
            status_str += f"{param_name}: {p}\n"
        print(f"Status OK: \n{status_str}")
        
        if publishPackets:
            publish("arduino/recv", (res.packetType, res.command, params))
    else:
        print(f"Arduino sent unknown response type {res_type}")

def handleErrorResponse(res: TPacket , publishPackets:bool=False):
    """
    Handles error responses from the Arduino.

    This function takes a TPacket object, determines the type of error response
    from the Arduino, and prints an appropriate error message.

    Args:
        res (TPacket): The packet received from the Arduino containing the error response.

    Raises:
        ValueError: If the response type is unknown.
    """
    res_type = TResponseType(res.command)
    if res_type == TResponseType.RESP_BAD_PACKET:
        print("Arduino received bad magic number")
    elif res_type == TResponseType.RESP_BAD_CHECKSUM:
        print("Arduino received bad checksum")
    elif res_type == TResponseType.RESP_BAD_COMMAND:
        print("Arduino received bad command")
    elif res_type == TResponseType.RESP_BAD_RESPONSE:
        print("Arduino received unexpected response")
    else:
        print(f"Arduino reports unknown error type {res_type}")

    if publishPackets:
        publish("arduino/recv", (res.packetType, res.command ))

def handleMessage(res:TPacket, publishPackets:bool=False):
    """
    Handles the incoming message from an Arduino device.

    Args:
        res (TPacket): The packet received from the Arduino, containing the data to be processed.

    Returns:
        None
    """
    message = str(res.data, 'utf-8')
    print(f"Arduino says: {message}")
    if publishPackets:
        publish("arduino/recv", (res.packetType, res.command, message))
    pass

