# This node is an example of a simple consumer/subscriber that subscribes to any topic/root topic and prints the received messages.

# Import Python Native Modules. We require the Barrier class from threading to synchronize the start of multiple threads.
from threading import Barrier

# Import the required pubsub modules. PubSubMsg class to extract the payload from a message.
from pubsub.pub_sub_manager import ManagedPubSubRunnable, PubSubMsg
from pubsub.pub_sub_manager import publish, subscribe, unsubscribe, getMessages, getCurrentExecutionContext


# constants
TOPICS_TO_MONITOR = ["arduino/send",] # Right now we are monitoring only the arduino send topic
MAX_PAYLOAD_PRINT_LENGTH = 50

def monitorThread(setupBarrier:Barrier=None, readyBarrier:Barrier=None, topicsToMonitor: list = TOPICS_TO_MONITOR):
    """
    Thread function to handle monitoring of messages on any topic.
    This function subscribes to any topic or root topic and prints the received messages.
    
    Args:
        readyBarrier (Barrier, optional): A threading barrier to synchronize the start of the thread. Defaults to None.
    Raises:
        Exception: If there is an error during message processing.
    Returns:
        None
    """
    # Setup
    ctx:ManagedPubSubRunnable = getCurrentExecutionContext()

    # Perform any setup here
    setupBarrier.wait() if readyBarrier != None else None
    
    # Subscribe to the root topic
    for t in topicsToMonitor:
        subscribe(topic=t, ensureReply=True, replyTimeout=1)
        print(f"Monitoring {t}")

    print(f"Monitor Thread Ready. Will monitor messages from {topicsToMonitor}")

    # Wait for all Threads ready
    readyBarrier.wait() if readyBarrier != None else None

    # Receiving Logic Loop
    # Message Monitor Loop
    try:
        while(not ctx.isExit()):
            m = getMessages(block=True, timeout=1)
            if m:
                for x in m:
                    t = PubSubMsg.getTopic(x)
                    s = PubSubMsg.getSender(x)
                    p = str(PubSubMsg.getPayload(x))

                    # Truncate the payload if it is too long
                    if len(p) > MAX_PAYLOAD_PRINT_LENGTH:
                        p = p[:MAX_PAYLOAD_PRINT_LENGTH] + "..."

                    print(f"Monitor: \n\tTopic: {t}\n\tSender: {s}\n\tPayload:  {p}")
    except Exception as e:
        print(f"Monitor Thread Exception: {e}")
        pass
    except KeyboardInterrupt:
        pass
    


    # Shutdown and exit the thread gracefully
    ctx.doExit()
    print("Exiting Monitor Thread")
    pass