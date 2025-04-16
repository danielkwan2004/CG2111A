"""
This module provides serialization and deserialization functions for communication packets.

Functions:
    calc_checksum(d: bytes, size: int = COMMS_BUFFER_SIZE) -> int:
        Calculates the checksum for a given byte sequence.
    deserialize(d: bytes, magic_number: int = COMMS_MAGIC_NUMBER) -> Tuple[TResultType, Optional[bytes]]:
        Deserializes a byte sequence into a TComms object and verifies its integrity using a magic number and checksum.
    serialize(payload: TPacket, magic_number: int = COMMS_MAGIC_NUMBER) -> bytes:
        Serializes a TPacket object into a byte sequence, including a magic number and checksum for integrity verification.
"""

from ctypes import memmove, pointer, Array, c_char
import ctypes
from .alex_control_constants import TComms, TResultType,  COMMS_MAGIC_NUMBER, COMMS_PACKET_SIZE, COMMS_BUFFER_SIZE
from .alex_control_constants import TPacket, TResponseType, PAYLOAD_PACKET_SIZE

def calc_checksum(d:bytes, size = COMMS_BUFFER_SIZE):
    """
    Calculate the checksum of a given byte sequence.

    Args:
        d (bytes): The byte sequence to calculate the checksum for.
        size (int, optional): The number of bytes to include in the checksum calculation. Defaults to COMMS_BUFFER_SIZE.

    Returns:
        int: The calculated checksum as an integer.
    """
    chk = 0b0
    for i in range(size):
        chk ^= d[i]
    return chk  

def deserialize(d:bytes, magic_number=COMMS_MAGIC_NUMBER):
    """
    Deserialize a byte stream into a communication packet and validate its integrity.
    Args:
        d (bytes): The byte stream to deserialize.
        magic_number (int, optional): The expected magic number for the packet. Defaults to COMMS_MAGIC_NUMBER.
    Returns:
        tuple: A tuple containing:
            - TResultType: The result type indicating the status of the deserialization.
            - payload (bytes or None): The payload of the packet if deserialization is successful, otherwise None.
    """
    comms = TComms.from_buffer_copy(d)
    payload = comms.buffer

    if comms.magic != magic_number:
        return TResultType.PACKET_BAD, None
    
    chk = calc_checksum(payload, size = comms.dataSize)

    if comms.checksum != chk:
        print(f"Checksum mismatch: {comms.checksum} (received) != {chk} (calculated)")
        return TResultType.PACKET_CHECKSUM_BAD, payload
    
    return TResultType.PACKET_OK, payload

def serialize(payload:TPacket, magic_number=COMMS_MAGIC_NUMBER):
    """
    Serializes a given payload into byte stream with a specified magic number.
    Args:
        payload (TPacket): The payload to be serialized.
        magic_number (int, optional): The magic number to be used in the serialization. Defaults to COMMS_MAGIC_NUMBER.
    Returns:
        bytes: The serialized byte stream of the payload.
    Raises:
        ValueError: If the payload size exceeds the buffer size.
    Notes:
        - The function calculates a checksum for the payload and includes it in the serialized data.
        - The serialized data includes a magic number, data size, payload bytes, and checksum.
    """
    
    # Serialize the payload into a C array
    payload_bytes = ctypes.ARRAY(ctypes.c_uint8, COMMS_BUFFER_SIZE)()
    memmove(pointer(payload_bytes), pointer(payload), PAYLOAD_PACKET_SIZE)

    # Calculate checksum
    dataSize = len(payload_bytes)
    chk = calc_checksum(payload_bytes, size = dataSize)

    # Wrap the payload in a communication packet
    comms = TComms()
    comms.magic = magic_number
    comms.dataSize = dataSize
    comms.buffer = payload_bytes
    comms.checksum = chk

    # Serialize the communication packet
    return bytes(comms)