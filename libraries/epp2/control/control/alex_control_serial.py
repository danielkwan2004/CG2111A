"""
This module provides functions to manage a serial connection using the `pyserial` library.
Functions:
    startSerial(portName: str, baudRate: int, byteSize: int, parity: str, stopBits: int, maxAttempts: int, serialTimeout: int = None, failedAttemptWaitSec: int = 5) -> bool:
        Attempts to open a serial connection with the specified parameters. Retries up to `maxAttempts` times if the connection fails.
    readSerial(bytes_to_read: int) -> tuple:
        Reads a specified number of bytes from the serial connection. Returns a tuple containing the number of bytes read and the data.
    writeSerial(b: bytes) -> int:
        Writes a byte sequence to the serial connection. Returns the number of bytes written.
    closeSerial() -> bool:
        Closes the serial connection if it is open. Returns True if the connection was successfully closed, False otherwise.
"""

import serial, time
from .alex_control_constants import PAYLOAD_PACKET_SIZE

##################################
### SINGLETON STATE VARIABLES ####
##################################
_MYSERIAL = None
_VERBOSE = True

##################################
###   SERIAL PORT FUNCTIONS   ####
##################################

def startSerial(
        portName: str, baudRate: int, byteSize: int, parity: str, stopBits: int, maxAttempts: int,
        serialTimeout: int = None, failedAttemptWaitSec: int = 5):
    """
    Attempts to open a serial port connection with the specified parameters.
    Args:
    portName (str): The name of the serial port to connect to.
    baudRate (int): The baud rate for the serial communication.
    byteSize (int): The byte size for the serial communication.
    parity (str): The parity for the serial communication.
    stopBits (int): The number of stop bits for the serial communication.
    maxAttempts (int): The maximum number of attempts to try connecting to the serial port.
    serialTimeout (int, optional): The timeout for the serial communication. Defaults to None.
    failedAttemptWaitSec (int, optional): The number of seconds to wait between failed attempts. Defaults to 5.
    Returns:
    bool: True if the serial port was successfully opened, False otherwise.
    """
    global _MYSERIAL
    if (_MYSERIAL != None):
        print ("Serial port already open") if _VERBOSE else None
        return False
    
    attempt = 0
    while (attempt<maxAttempts):
        try:
            print(f"Attempting to connect to Serial port {portName}... Attempt {attempt} of {maxAttempts}") if _VERBOSE else None
            ser = serial.Serial(
                port=portName,
                baudrate=baudRate,
                bytesize=byteSize,
                parity=parity,
                stopbits=stopBits,
                timeout=serialTimeout
            )
            ser.close()
            ser.open()
            _MYSERIAL = ser
            print(f"Connected to Serial port {portName}") if _VERBOSE else None
            return True
        except Exception as e:
            print (f"Failed to open serial port, Trying again in {failedAttemptWaitSec} seconds ") if _VERBOSE else None
            print (e) if _VERBOSE else None
            attempt += 1
            time.sleep(5)
            continue
    
    print (f"Failed to open serial port after {attempt} attempts. Giving up") if _VERBOSE else None
    return False

def readSerial(bytes_to_read:int):
    """
    Reads a specified number of bytes from the serial port.

    Args:
        bytes_to_read (int): The number of bytes to read from the serial port.

    Returns:
        tuple: A tuple containing the number of bytes read and the data read from the serial port.
                If the serial port is not open, returns (0, None).
    """
    global _MYSERIAL
    if (_MYSERIAL == None):
        print ("Serial port not open") if _VERBOSE else None
        return 0, None
    res = _MYSERIAL.read(bytes_to_read)
    return len(res), res

def writeSerial(b:bytes):
    """
    Write bytes to the serial port.

    This function writes the given bytes to the serial port and flushes the output buffer.
    If the serial port is not open, it prints an error message (if verbose mode is enabled)
    and returns False. If the input is not of type bytes, it raises a ValueError.

    Args:
        b (bytes): The bytes to be written to the serial port.

    Returns:
        int: The number of bytes written to the serial port if successful.
        bool: False if the serial port is not open.

    Raises:
        ValueError: If the input is not of type bytes.
    """
    global _MYSERIAL
    if (_MYSERIAL == None):
        print ("Serial port not open") if _VERBOSE else None
        return False
    if (type(b) != bytes):
        raise ValueError("Input must be of type bytes")
    _MYSERIAL.write(b)
    _MYSERIAL.flush()
    return len(b)

def closeSerial():
    """
    Closes the serial port if it is open.

    This function checks if the global serial port object `_MYSERIAL` is open.
    If it is open, it closes the serial port and sets `_MYSERIAL` to `None`.
    If the serial port is not open, it prints a message if `_VERBOSE` is set to `True`.

    Returns:
        bool: `True` if the serial port was successfully closed, `False` if the serial port was not open.
    """
    global _MYSERIAL
    if (_MYSERIAL == None):
        print ("Serial port not open") if _VERBOSE else None
        return False
    # if already closed
    if not _MYSERIAL.is_open:
        _MYSERIAL = None
        return True
    else:
        _MYSERIAL.close()
        _MYSERIAL = None
        return True
