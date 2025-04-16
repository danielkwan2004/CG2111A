"""
This module wraps Python's built-in SSL library to provide functions for managing Secure Socket Layer (SSL) connections.
It builds a wrapper around the ssl module from the Python standard library and implements functionality to set up secure connections,
send and receive network data, and gracefully terminate connections.

Exposed API:
    connect(host: str, port: int, client_key_path: str, client_cert_path: str, server_canonical_name: str, ca_cert_path: str, timeout: int = 1) -> bool
    disconnect() -> bool
    sendNetworkData(data: bytes) -> int
    recvNetworkData(bufsize: int = 4096) -> bytes
"""
import socket,ssl,os

VERBOSE = False
print = print if VERBOSE else lambda *a, **k: None

# Define a default SSL context for creating secure connections
_TLSConnection = None

def isTLSConnected() -> bool:
    """
    Check if a TLS connection is currently established.

    Returns:
        bool: True if a TLS connection exists, False otherwise.
    """
    return _TLSConnection is not None

def getTLSConnection() -> ssl.SSLSocket:
    """
    Retrieve the current TLS connection.

    Returns:
        ssl.SSLSocket: The active TLS connection, or None if not connected.
    """
    return _TLSConnection

def setTLSConnection(ssl_socket: ssl.SSLSocket):
    """
    Set the global TLS connection.

    Args:
        ssl_socket (ssl.SSLSocket): The TLS socket to store.
    """
    global _TLSConnection
    _TLSConnection = ssl_socket

def connect(
        # Server address and port
        host: str, port: int, 

        # Client certificate and key
        client_key_path: str,
        client_cert_path: str,

        # Server Name on the certificate
        server_canonical_name: str, 

        # CA certificate
        ca_cert_path: str,

        # timeout in seconds
        timeout: int = 1

        ) -> ssl.SSLSocket:
    """
    Establish a secure SSL connection to the specified server using the provided credentials and certificate.

    Args:
        host (str): The server hostname or IP address.
        port (int): The server port number.
        client_key_path (str): Path to the client's private key file.
        client_cert_path (str): Path to the client's certificate file.
        server_canonical_name (str): The expected server name on the certificate.
        ca_cert_path (str): Path to the CA certificate file for verifying the server.
        timeout (int, optional): Connection timeout in seconds. Defaults to 1.

    Returns:
        bool: True if the connection was successfully established, False otherwise.
    """
    # if already connected, return
    if isTLSConnected():
        return True

    # Create a plain socket and wrap it with SSL
    sslcontext = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    sslcontext.load_verify_locations(ca_cert_path)
    sslcontext.load_cert_chain(client_cert_path, client_key_path)
    sslcontext.check_hostname = True
    sslcontext.verify_mode = ssl.CERT_REQUIRED

    # Create a secure SSL connection
    ssl_socket = sslcontext.wrap_socket(socket.socket(socket.AF_INET, socket.SOCK_STREAM), server_hostname=server_canonical_name)

    # Set socket settings
    # Timemout mode, Automatically also sets the socket to "non-blocking" mode
    ssl_socket.settimeout(timeout)

    # Connect to the server
    try:
        ssl_socket.connect((host, port))
        setTLSConnection(ssl_socket)
    # Handle connection errors
    except ssl.SSLError as e:
        print(f"SSL Error: {e}")
        setTLSConnection(None)
        return False
    except socket.timeout as e:
        print(f"Socket Timeout: {e}")
        setTLSConnection(None)
        return False
    except Exception as e:
        print(f"Error: {e}")
        setTLSConnection(None)
        return False
    return True

def disconnect() -> bool:
    """
    Gracefully disconnect the active TLS connection.

    Returns:
        bool: True if successfully disconnected (or if no active connection existed), False otherwise.
    """
    hasConnection = isTLSConnected()
    if not hasConnection:
        return True

    ssl_socket = getTLSConnection()
    try:
        ssl_socket.shutdown(socket.SHUT_RDWR)
        ssl_socket.close()
        setTLSConnection(None)
    except ssl.SSLError as e:
        print(f"SSL Error: {e}")
        setTLSConnection(None)
        return False
    except socket.timeout as e:
        print(f"Socket Timeout: {e}")
        setTLSConnection(None)
        return False
    except Exception as e:
        print(f"Error: {e}")
        setTLSConnection(None)
        return False
    return


def sendNetworkData(data: bytes) -> int:
    """
    Send data over the active TLS connection. Does not handle endianness.

    Args:
        data (bytes): The data to be transmitted.

    Returns:
        int: The number of bytes sent if successful; -1 on failure.
    """
    ssl_socket = getTLSConnection()
    if not ssl_socket:
        return -1
    try:
        print(f"Sending {len(data)} bytes")
        return  ssl_socket.send(data) 
    except ssl.SSLError as e:
        print(f"SSL Error: {e}")
        return -1
    except socket.timeout as e:
        print(f"Socket Timeout: {e}")
        return -1

def recvNetworkData(bufsize: int = 4096) -> tuple[bytes, int]:
    """
    Receive data from the active TLS connection.  Does not handle endianness.

    Args:
        bufsize (int, optional): Maximum number of bytes to receive. Defaults to 4096.

    Returns:
        tuple: A tuple containing:
            - bytes: The received data if successful; None on error.
            - int: The number of bytes received, or 0 if a timeout occurred, or -1 on error.
    """
    ssl_socket = getTLSConnection()
    if not ssl_socket:
        return None, -1

    try:
        recvData = ssl_socket.recv(bufsize)
        dataSize = len(recvData)
        dataSize = dataSize if dataSize > 0 else -1 
        return recvData, dataSize 
    except socket.timeout as e:
        print(f"Socket Timeout: {e}")
        return None, 0
    except ssl.SSLError as e:
        print(f"SSL Error: {e}")
        return None , -1
    except Exception as e:
        print(f"Error: {e}")
        return None, -1
