# This node is an example of a simple consumer that subscribes to the 
# arduino/send topic and sends all messages it receives to the arduino

# Import Python Native Modules. We require the Barrier class from threading to synchronize the start of multiple threads.
from threading import Barrier

# Import the required pubsub modules. PubSubMsg class to extract the payload from a message.
from pubsub.pub_sub_manager import ManagedPubSubRunnable, PubSubMsg
from pubsub.pub_sub_manager import publish, subscribe, unsubscribe, getMessages, getCurrentExecutionContext  

# Import the required arduino communication modules. We require the sendPacket function to send messages to the arduino.
from control.alex_control import sendPacket

# Constants
ARDUINO_SEND_TOPIC = "arduino/send"


# Thread target function
def sendThread(setupBarrier:Barrier=None, readyBarrier:Barrier=None):
    """
    Function to handle sending Arduino messages in a thread.
    It listens for messages on the "arduino/send" topic and sends the corresponding packets to the Arduino.

    Args:
        setupBarrier (Barrier, optional): A threading barrier to synchronize setup among threads. Defaults to None.
        readyBarrier (Barrier, optional): A threading barrier to synchronize the start of multiple threads. Defaults to None.

    Steps:
        1. Performs any initial setup.
        2. Retrieves the current execution context.
        3. Waits for the setup barrier if provided.
        4. Subscribes to the "arduino/send" topic with a reply timeout of 10 seconds.
        5. Waits for the ready barrier if provided.
        6. Enters a loop to continuously check for messages and send packets until signaled to exit.
        7. Gracefully shuts down the thread once the execution context signals exit.
    """
    # Perform any setup here
    pass

    # Get the current execution context (i.e., is this running in the main process or a thread or a subprocess)
    ctx:ManagedPubSubRunnable = getCurrentExecutionContext()

    
    

    # Perform any setup here
    setupBarrier.wait() if readyBarrier != None else None
    
    # Subscribe to the arduino/send topic. We make sure to block until we get a reply
    r = subscribe(topic=ARDUINO_SEND_TOPIC, ensureReply=True, replyTimeout=10)
    print(f"Arduino Send Thread Ready. Subscribed to {ARDUINO_SEND_TOPIC} --> {r}")

    # Wait for all Threads ready
    readyBarrier.wait() if readyBarrier != None else None
    
    # Main Logic Loop
    try:
        while(not ctx.isExit()):
            # Get any remaining messages
            messages = getMessages(block=True, timeout=1)

            # Handle each message
            for m in messages:
                payload = PubSubMsg.getPayload(m)
                # we expect the payload to be a tuple of:
                # - packetType: enum
                # - commandType: enum
                # - params: list

                packetType, commandType, params = payload
                sendPacket(packetType, commandType, params)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Send Thread Exception: {e}")
        pass

    # Shutdown and exit the thread gracefully
    ctx.doExit()
    print("Exiting Send Thread")
    pass
