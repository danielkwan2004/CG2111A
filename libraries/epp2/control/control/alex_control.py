"""
This module provides functions to handle communication with an Arduino device over a serial connection.
It includes serial communication functions, user input parsing, and routines.
Functions:
    receivePacket(exitFlag: Event = _EXIT_EVENT) -> TPacket:
        Receives a packet from the Arduino over the serial connection.
    sendPacket(packetType: TPacketType, commandType: TCommandType, params: list):
        Sends a packet to the Arduino over the serial connection.
    printPacket(packet: TPacket):
        Prints the details of a packet.
    parseParams(p: list, num_p: int, inputMessage: str) -> list:
        Parses parameters for a command.
    parseUserInput(input_str: str, exitFlag: Event = _EXIT_EVENT, sendPacket = sendPacket):
        Parses user input and sends the corresponding packet to the Arduino.
    waitForHelloRoutine():
        Waits for a hello response from the Arduino.
"""
from threading import Event
from time import sleep
from .alex_control_serial import startSerial, readSerial, writeSerial, closeSerial
from .alex_control_serialize import serialize, deserialize
from .alex_control_constants import TCommandType, TPacketType, TResponseType, TPacket, PAYLOAD_PARAMS_COUNT, PAYLOAD_PACKET_SIZE
from .alex_control_constants import TComms, TResultType, COMMS_MAGIC_NUMBER, COMMS_PACKET_SIZE


# This default event is used to signal that the program should exit
# Pass your own event to the parseUserInput() and receivePacket() functions if you want to control the exit behavior
_EXIT_EVENT = Event()


############################
## SERIAL COMMS FUNCTIONS ##
############################
def receivePacket(exitFlag:Event=_EXIT_EVENT):
    """
    Receives a packet from the serial interface.

    This function continuously reads from the serial interface until a complete packet
    is received or the exit flag is set. The packet is then deserialized and returned.

    Args:
        exitFlag (Event, optional): An event flag to signal when to exit the loop. Defaults to _EXIT_EVENT.

    Returns:
        TPacket: The deserialized packet if received and valid.
        None: If the packet is invalid or if the exit flag is set before a complete packet is received.

    Raises:
        Any exceptions raised by readSerial or deserialize functions.
    """
    target_packet_size = COMMS_PACKET_SIZE
    buffer = bytearray(target_packet_size)
    buffer_size = 0
    while(not exitFlag.is_set()):
        res_size, res = readSerial(target_packet_size-buffer_size)
        # print(f"Received {res_size} bytes") if VERBOSE else None
        if res_size != 0:
            buffer[buffer_size:buffer_size+res_size] = res
            buffer_size += res_size
        if buffer_size == target_packet_size:
            # deserialize packet
            res_status, payload = deserialize(buffer)
            if res_status == TResultType.PACKET_OK:
                return TPacket.from_buffer(payload)
            else:
                handleError(res_status)
                return None
    return None

def sendPacket(packetType:TPacketType, commandType:TCommandType, params:[]):
    """
    Sends a packet with the specified type, command, and parameters.

    Args:
        packetType (TPacketType): The type of the packet to send. Can be an instance of TPacketType or an integer.
        commandType (TCommandType): The command type of the packet. Can be an instance of TCommandType or an integer.
        params (list): A list of parameters to include in the packet. Should be a list of integers.

    Returns:
        None
    """
    packet_to_send = TPacket()
    packet_to_send.packetType = int(packetType.value) if isinstance(packetType, TPacketType) else int(packetType)
    packet_to_send.command = int(commandType.value) if isinstance(commandType, TCommandType) else int(commandType)
    if (params != []):
        packet_to_send.params[0:PAYLOAD_PARAMS_COUNT] = [int(x) for x in params]
    to_comms = serialize(packet_to_send)
    # print(f"Sending Packet.\nSize: {len(to_comms)}\nData: {to_comms}") if VERBOSE else None
    writeSerial(to_comms) 

def handleError(res_status:TResultType):
    """
    Handles errors based on the result status type.

    Parameters:
    res_status (TResultType): The result status type indicating the error condition.

    Returns:
    None

    Prints:
    - "ERROR: Received Bad Packet from Arduino" if the result status is PACKET_BAD.
    - "ERROR: Received Bad Checksum from Arduino" if the result status is PACKET_CHECKSUM_BAD.
    - "ERROR: Unknown Error in Processing Packet" for any other result status.
    """
    if res_status == TResultType.PACKET_BAD:
        print("ERROR: Received Bad Packet from Arduino")
    elif res_status == TResultType.PACKET_CHECKSUM_BAD:
        print("ERROR: Received Bad Checksum from Arduino")
    else:
        print("ERROR: Unknown Error in Processing Packet")


def printPacket(packet:TPacket):
    """
    Prints the details of a TPacket object.

    Args:
        packet (TPacket): The packet object to be printed. It should have the following attributes:
            - packetType: The type of the packet.
            - command: The command associated with the packet.
            - data: The data contained in the packet.
            - params: A list of parameters associated with the packet.
    """
    print(f"Packet Type: {packet.packetType}")
    print(f"Command: {packet.command}")
    print(f"Data: {packet.data}")
    params = [x for x in packet.params]
    print(f"Params: {params}")


##########################
## USER INPUT FUNCTIONS ##
##########################

def parseParams(p, num_p, inputMessage):
    """
    Parses and returns a list of parameters based on the given input. If the number of parameters is less than `num_p`,
    the function will prompt the user for additional input, and will block until the user enters a valid input.

    Args:
        p (list): A list of initial parameters.
        num_p (int): The number of parameters expected.
        inputMessage (str): A message to prompt the user for input if needed.

    Returns:
        list: A list of parameters with a length of PAYLOAD_PARAMS_COUNT. If the number of parameters
              in `p` is less than `num_p`, the function will prompt the user for additional input.
              If `num_p` is 0, returns a list of zeros with length PAYLOAD_PARAMS_COUNT.
              If the input is invalid, returns None.
    """
    if (num_p == 0):
        return [0]*PAYLOAD_PARAMS_COUNT
    elif (len(p) >= num_p):
        return  p[:num_p] + [0]*(PAYLOAD_PARAMS_COUNT - num_p)
    elif (len(p) < num_p) and (inputMessage != None):
        params_str = input(inputMessage)
        split_input = params_str.split(" ")
        return parseParams(split_input, num_p, None)
    else:
        return None
        
def parseUserInput(input_str:str, exitFlag:Event = _EXIT_EVENT):
    """
    Parses the user input string and executes the corresponding command. This function blocks until the user enters a valid command. 
    Args:
        input_str (str): The input string from the user.
        exitFlag (Event, optional): An event flag to signal exit. Defaults to _EXIT_EVENT.
        transmitCommand (function, optional): Function to send the packet. Defaults to sendPacket.
    Returns:
        tuple: A tuple containing the packet type, command type, and parameters if the input is valid.
        None: If the input is invalid or the exit flag is set.
    Commands:
        f: Move forward. Requires distance in cm and power in %.
        b: Move backward. Requires distance in cm and power in %.
        l: Turn left. Requires degrees to turn and power in %.
        r: Turn right. Requires degrees to turn and power in %.
        s: Stop the movement.
        c: Clear statistics.
        g: Get statistics.
        q: Quit the program and set the exit flag.
    Example:
        parseUserInput("f 50 75")
        parseUserInput("q")
    """
    # split by space
    split_input = [x for x in input_str.strip().split(" ") if x != ""]
    
    # Handle Malformed Input
    if len(split_input) <1:
        return print(f"{input_str} is not a valid command")
    command = split_input[0]

    # Building Packet
    packetType = TPacketType.PACKET_TYPE_COMMAND

    if command == "f":
        commandType = TCommandType.COMMAND_FORWARD
        params = parseParams(split_input[1:], 2, 
                             "Enter distance in cm (e.g. 50) and power in % (e.g. 75) separated by space.\n")
        return (packetType, commandType,  params) if params != None else print("Invalid Parameters")
    elif command== "b":
        commandType = TCommandType.COMMAND_REVERSE
        params = parseParams(split_input[1:], 2, 
                             "Enter distance in cm (e.g. 50) and power in % (e.g. 75) separated by space.\n")
        return (packetType, commandType,  params) if params != None else print("Invalid Parameters")
    elif command == "l":
        commandType = TCommandType.COMMAND_TURN_LEFT
        params = parseParams(split_input[1:], 2, 
                             "Enter degrees to turn left (e.g. 90) and power in % (e.g. 75) separated by space.\n")
        return (packetType, commandType,  params) if params != None else print("Invalid Parameters")
    elif command == "r":
        commandType = TCommandType.COMMAND_TURN_RIGHT
        params = parseParams(split_input[1:], 2, 
                             "Enter degrees to turn right (e.g. 90) and power in % (e.g. 75) separated by space.\n")
        return (packetType, commandType,  params) if params != None else print("Invalid Parameters")
    elif command == "s":
        commandType = TCommandType.COMMAND_STOP
        params = parseParams(split_input[1:], 0, None)
        return (packetType, commandType,  params)
    elif command == "c":
        commandType = TCommandType.COMMAND_CLEAR_STATS
        params = parseParams(split_input[1:], 0, None)
        return  (packetType, commandType,  params)
    elif command == "g":
        commandType = TCommandType.COMMAND_GET_STATS
        params = parseParams(split_input[1:], 0, None)
        return  (packetType, commandType,  params)
    elif command == "q":
        print("Exiting! Setting Exit Flag...")
        print("\n==============CLEANING UP==============")
        exitFlag.set()
        return None
    else:
        return print(f"{command} is not a valid command")


###########################
######## Routines #########
###########################
def waitForHelloRoutine():
    """
    Sends a "Hello" packet to the Arduino and waits for a response.
    Waits for a "Hello" response packet from the Arduino. This is the intial handshake between the host and the Arduino, and is used to confirm that the Arduino is ready to receive commands.

    This function waits for a packet from the Arduino and checks if the packet
    type is a response and the command type is RESP_OK. If the correct packet
    is received, it prints a confirmation message. If the packet type or command
    type does not match the expected values, it raises an exception.
    
    Raises:
        Exception: If the received packet type or command type does not match
                   the expected values.
    Returns:
        None
    """
    sendPacket(TPacketType.PACKET_TYPE_HELLO, TCommandType.COMMAND_STOP, [0]*PAYLOAD_PARAMS_COUNT)
    packet = receivePacket()
    if packet != None:
        packetType = packet.packetType
        res_type = packet.command
        if (
            packetType == TPacketType.PACKET_TYPE_RESPONSE.value and
            res_type == TResponseType.RESP_OK.value
        ):
            print("Received Hello Response")
            return 

    raise Exception(f"Failed to receive proper response from Arduino: Packet Type Mismatch {packetType}")
