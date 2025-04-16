"""
This module wraps Python's built-in SSL library to provide functions for managing Secure Socket Layer (SSL) server connections.
It builds upon the ssl module from the Python standard library and implements functionality to set up secure server connections,
accept incoming TLS connections, send and receive network data, and gracefully terminate connections.

Note: Implements a Singleton pattern to manage the server and connection state. This means that only one server can be active at a time in a single process. But across multiple processess..... Who knows? who nose......

Note2: Do you really need more than one server? You CAN handle multiple clients with one server.

Exposed API:
    setupTLSServer(bind_host: str, port: int, server_key_path: str, server_cert_path: str, ca_cert_path: str, expected_client_name: str) -> bool
    disconnect() -> bool
    sendNetworkData(data: bytes) -> int
    recvNetworkData(bufsize: int = 4096) -> (bytes, int)
"""
import socket, ssl, os

VERBOSE = False
print = print if VERBOSE else lambda *a, **k: None

# Connection
_TLSConnection = None

# Server socket and expected client name
_Server = None
_clientName = None
_SSLContext = None

def isServerAlive() -> bool:
    """
    Check if the server is currently running.

    Returns:
        bool: True if the server is running, False otherwise.
    """
    return _Server is not None

def setServer(socket: socket.socket, clientName: str, SSLContext: ssl.SSLContext):
    """
    Set the global server socket, expected client name, and SSL context.

    Args:
        socket (socket.socket): The plain socket used for accepting incoming connections.
        clientName (str): The expected client name to validate the client's certificate.
        SSLContext (ssl.SSLContext): The SSL context configured for establishing secure connections.
    """
    global _Server, _clientName, _SSLContext
    _Server = socket
    _clientName = clientName
    _SSLContext = SSLContext
    return 

def getServer() :
    """
    Retrieve the current TLS server, the expected client name, and the SSL context.

    Returns:
        tuple: A tuple containing:
            - socket.socket: The active TLS server, or None if not connected.
            - str: The expected client name associated with the server.
            - ssl.SSLContext: The SSL context used by the server.
    """
    return _Server, _clientName, _SSLContext  


def isTLSConnected() -> bool:
    """
    Check if a TLS connection is currently established.

    Returns:
        bool: True if a TLS connection exists, False otherwise.
    """
    return (_TLSConnection is not None) and (_Server is not None)

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

def getPeerDetails() -> tuple[str, int]:
    """
    Get the address and port of the connected client.

    Returns:
        tuple: A tuple containing:
            - str: The IP address of the connected client.
            - int: The port number of the connected client.
    """
    if not isTLSConnected():
        return None, None
    return _TLSConnection.getpeername()


def setupTLSServer(
        bind_host: str, port: int,
        server_key_path: str,
        server_cert_path: str,
        ca_cert_path: str,
        expected_client_name: str,
    ) -> bool:
    """
    Set up the TLS server by creating and configuring an SSL context along with a listening socket.
    This function binds to the specified host and port, loads the server's certificate and key,
    and configures client certificate verification using the provided CA certificate.
    
    Args:
        bind_host (str): The hostname or IP address to bind the server.
        port (int): The port number to listen on.
        server_key_path (str): Path to the server's private key file.
        server_cert_path (str): Path to the server's certificate file.
        ca_cert_path (str): Path to the CA certificate file for client verification.
        expected_client_name (str): The expected hostname for the connecting client as per its certificate.
    
    Returns:
        bool: True if the TLS server is successfully set up; otherwise False.
    """
    if isTLSConnected():
        print("TLS connection already established")
        return False

    try:
        # Create SSL context for server-side with client certificate authentication
        sslcontext = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        sslcontext.load_cert_chain(certfile=server_cert_path, keyfile=server_key_path)
        sslcontext.load_verify_locations(ca_cert_path)
        sslcontext.verify_mode = ssl.CERT_REQUIRED

        # Create a plain socket, bind it, and listen for incoming connections
        bindsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        bindsocket.settimeout(1)
        bindsocket.bind((bind_host, port))
        bindsocket.listen(1)
        print(f"Server listening on {bind_host}:{port}...")

        # Store the server socket and expected client name
        setServer(bindsocket, expected_client_name, sslcontext)
        return True
    except ssl.SSLError as e:
        print(f"SSL Error: {e}")
        return False
    except socket.timeout as e:
        print(f"Socket Timeout: {e}")
        return False
    
def shutdownServer():
    """
    Shutdown the server socket and close all connections.
    """
    global _Server, _clientName, _SSLContext

    # kill client
    if isTLSConnected():
        disconnect()

    if _Server:
        print("Shutting down server")
        _Server.shutdown(socket.SHUT_RDWR)
        _Server.close()
    _Server = None
    _clientName = None
    _SSLContext = None

def disconnect() -> bool:
    """
    Gracefully disconnect the active TLS connection.

    Returns:
        bool: True if successfully disconnected (or if no active connection existed), False otherwise.
    """
    if not isServerAlive():
        return True

    if not isTLSConnected():
        return True

    print("Disconnecting TLS Connection")
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
    return True

def acceptTLSConnection(timeout:int = 1) -> bool:
    """
    Accept an incoming TLS connection request. And check if the client name matches the expected client name using ssl.getpeercert() and ssl.match_hostname().

    Returns:
        bool: True if the connection was successfully accepted, False otherwise.
    """
    if not isServerAlive():
        print("Server is not running")
        return False

    if isTLSConnected():
        print("TLS Connection already established")
        return False

    server, clientName, sslcontext = getServer()
    try:
        # Accept incoming connection
        server.settimeout(timeout)
        conn, addr = server.accept()
        print(f"Connection from {addr}")

        # Wrap the connection with SSL
        ssl_socket = sslcontext.wrap_socket(conn, server_side=True)
        peer_cert = ssl_socket.getpeercert()
        if not peer_cert:
            print("No client certificate provided")
            return False
        
        # Check if the client name matches the expected client name
        try:
            ssl.match_hostname(peer_cert, clientName)
        except ssl.CertificateError as e:
            print("Expected Client Name: ", clientName)
            print(f"GOT: {peer_cert}")
            print("Client Name does not match the expected name")
            return False
        
        # set the socket timeout
        ssl_socket.settimeout(timeout)

        # Store the TLS connection
        setTLSConnection(ssl_socket)
        print("TLS Connection Established")
        return True
    
    except ssl.SSLError as e:
        print(f"SSL Error: {e}")
        return False
    except socket.timeout as e:
        print(f"Socket Timeout: {e}")
        return False

    

def sendNetworkData(data: bytes) -> int:
    """
    Send data over the active TLS connection.  Does not handle endianness.

    Args:
        data (bytes): The data to be transmitted.

    Returns:
        int: The number of bytes sent if successful; -1 on failure.
    """
    ssl_socket = getTLSConnection()
    if not ssl_socket:
        return -1
    try:
        return ssl_socket.send(data)
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
        # Connection is dead
        return None, -1
    