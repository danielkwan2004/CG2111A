import enum
import ctypes

##########################
######### Enums ##########
##########################
class TPacketType(enum.Enum):
    """
    TPacketType is an enumeration that defines various types of packets used in the system.

    Attributes:
        PACKET_TYPE_COMMAND (int): Represents a command packet type.
        PACKET_TYPE_RESPONSE (int): Represents a response packet type.
        PACKET_TYPE_ERROR (int): Represents an error packet type.
        PACKET_TYPE_MESSAGE (int): Represents a message packet type.
        PACKET_TYPE_HELLO (int): Represents a hello packet type.
    """
    PACKET_TYPE_COMMAND = 0
    PACKET_TYPE_RESPONSE = 1
    PACKET_TYPE_ERROR = 2
    PACKET_TYPE_MESSAGE = 3
    PACKET_TYPE_HELLO = 4

class TResponseType(enum.Enum):
    """
    TResponseType is an enumeration that defines various types of response codes.

    Attributes:
        RESP_OK (int): Indicates a successful response.
        RESP_STATUS (int): Indicates a status response.
        RESP_BAD_PACKET (int): Indicates a response with a bad packet.
        RESP_BAD_CHECKSUM (int): Indicates a response with a bad checksum.
        RESP_BAD_COMMAND (int): Indicates a response with a bad command.
        RESP_BAD_RESPONSE (int): Indicates a response with a bad response.
    """
    RESP_OK = 0
    RESP_STATUS=1
    RESP_BAD_PACKET = 2
    RESP_BAD_CHECKSUM = 3
    RESP_BAD_COMMAND = 4
    RESP_BAD_RESPONSE = 5

class TCommandType(enum.Enum):
    """
    TCommandType is an enumeration that defines various command types for controlling a device.

    Attributes:
        COMMAND_FORWARD (int): Command to move the device forward.
        COMMAND_REVERSE (int): Command to move the device in reverse.
        COMMAND_TURN_LEFT (int): Command to turn the device to the left.
        COMMAND_TURN_RIGHT (int): Command to turn the device to the right.
        COMMAND_STOP (int): Command to stop the device.
        COMMAND_GET_STATS (int): Command to retrieve the device's statistics.
        COMMAND_CLEAR_STATS (int): Command to clear the device's statistics.
    """
    COMMAND_FORWARD = 0
    COMMAND_REVERSE = 1
    COMMAND_TURN_LEFT = 2
    COMMAND_TURN_RIGHT = 3
    COMMAND_STOP = 4
    COMMAND_GET_STATS = 5
    COMMAND_CLEAR_STATS = 6

class TResultType(enum.Enum):
    """
    TResultType is an enumeration that represents the possible outcomes of packet processing.

    Attributes:
        PACKET_OK (int): Indicates that the packet is okay.
        PACKET_BAD (int): Indicates that the packet is bad.
        PACKET_CHECKSUM_BAD (int): Indicates that the packet has a bad checksum.
        PACKET_INCOMPLETE (int): Indicates that the packet is incomplete.
        PACKET_COMPLETE (int): Indicates that the packet is complete.
    """
    PACKET_OK = 0
    PACKET_BAD = 1
    PACKET_CHECKSUM_BAD = 2
    PACKET_INCOMPLETE = 3
    PACKET_COMPLETE = 4

##########################
######## Packets #########
##########################
PAYLOAD_DATA_MAX_STR_LEN = 32
PAYLOAD_PARAMS_COUNT = 16
class TPacket(ctypes.Structure):
    """
    A class to represent a packet structure for sending and receiving data. This packet only contains the necessary fields for information exchange, but does not include fields for error checking or validation. Therefore, it is ,meant to used as a payload within a communication packet (i.e., TComms). 
    
    In communication and networking, this is called encapsulation, where a packet is encapsulated within another packet for transmission, where each layer adds its own header and footer information, and is responsible for providing different functionalities.

    Attributes
    ----------
    packetType : ctypes.c_uint8
        The type of the packet.
    command : ctypes.c_uint8
        The command associated with the packet.
    dummy : ctypes.c_char*2
        A dummy field for padding or alignment.
    data : ctypes.c_char*PAYLOAD_DATA_MAX_STR_LEN
        The data payload of the packet.
    params : ctypes.c_uint32*PAYLOAD_PARAMS_COUNT
        The parameters associated with the packet.
    """
    _pack_ = 1 # Set the alignment to 1 byte (no automatic padding)
    _fields_ = [
        ("packetType", ctypes.c_uint8),
        ("command", ctypes.c_uint8),
        ("dummy", ctypes.c_char*2),
        ("data", ctypes.c_char*PAYLOAD_DATA_MAX_STR_LEN),
        ("params", ctypes.c_uint32*PAYLOAD_PARAMS_COUNT),
    ]
PAYLOAD_PACKET_SIZE = ctypes.sizeof(TPacket)


# /* Data size is 4 + 128 + 4 + 1 = 137 bytes. We pad to 140 bytes as this is the nearest divisible by 4 we have. So 
# 	 we add 3 bytes */
COMMS_MAGIC_NUMBER = 0xFCFDFEFF
COMMS_BUFFER_SIZE = 128
class TComms(ctypes.Structure):
    """
    A ctypes Structure representing a communication packet. This structure is used for serializing and deserializing data for communication between devices, and includes fields for error checking and validation. 
    
    The Buffer field is used to store the arbitrary data payload (e.g., TPacket) that is being transmitted. The checksum field is used to validate the integrity of the data. The magic field is used to add a unique identifier to the packet for validation purposes.

    Attributes:
        magic (ctypes.c_uint32): A magic number used for validation.
        dataSize (ctypes.c_uint32): The size of the data in the buffer.
        buffer (ctypes.c_uint8 * COMMS_BUFFER_SIZE): A buffer to hold the data.
        checksum (ctypes.c_uint8): A checksum for error detection.
        dummy (ctypes.c_char * 3): A dummy field for padding.
    """
    _pack_ = 1 # Set the alignment to 1 byte (no automatic padding)
    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("dataSize", ctypes.c_uint32),
        ("buffer", ctypes.c_uint8*COMMS_BUFFER_SIZE),
        ("checksum", ctypes.c_uint8),
        ("dummy", ctypes.c_char*3),
    ]
COMMS_PACKET_SIZE = ctypes.sizeof(TComms)
        