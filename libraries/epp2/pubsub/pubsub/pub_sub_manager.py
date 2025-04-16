"""
This module provides a publish-subscribe (pub-sub) manager that supports nodes (e.g., Runnables ) in both threading and multiprocessing contexts.

It includes classes and functions to manage subscriptions, publications, and message handling across different execution contexts.

A node can be a thread or a process that subscribes to topics, publishes messages to topics, and receives messages from topics. These nodes allow users to run multiple threads or processes that can communicate with each other using pub-sub messaging.


Classes:
    PubSubMsg: Provides static methods to create and handle pub-sub messages.
    ManagedPubSubRunnable: Base class for managed pub-sub runnables.
    ManagedPubSubProcess: A managed pub-sub runnable implemented as a multiprocessing.Process.
    ManagedPubSubThread: A managed pub-sub runnable implemented as a threading.Thread.
    PubSubManager: Manages pub-sub operations, including adding processes and threads, starting and stopping them, and handling message routing.
Functions:
    getCurrentExecutionContext: Determines the current execution context (thread or process).
    subscribe: Subscribes to a given topic within the current execution context.
    unsubscribe: Unsubscribes from a given topic within the current execution context.
    publish: Publishes a message to a specified topic.
    getMessages: Retrieves messages from the current execution context.
Exceptions:
    Raises Exception if subscription, unsubscription, or publication is attempted from the main thread or from a non-managed runnable context.
"""

# Import multiprocessing classes
from multiprocessing import Process, current_process, parent_process
from multiprocessing import Queue as mQueue
from multiprocessing import Event as mEvent
# from multiprocessing.synchronize import Event as mEvent


# Import threading classes
from threading import Thread, current_thread
from threading import Event, Barrier, BrokenBarrierError
from queue import Queue, Empty, Full
# Import shared classes

import time


#####################
# Pub Sub Interface #
#####################
# The following functions can be called from any thread or process to interact with the pub-sub system.
# They will automatically determine the current execution context and route the request accordingly.

def getCurrentExecutionContext():
    """
    Determines the current execution context, which can be either a thread or a process.
    Returns:
        threading.Thread or multiprocessing.Process or None: 
            - The current thread if it is not the main thread.
            - The current process if it is a subprocess.
            - None if it is the main thread and not a subprocess.
    """
    curr_t = current_thread()
    # check if its the main thread
    if curr_t.name != 'MainThread':
       return curr_t
    
    # check if its a sub process
    parent_p = parent_process()
    if not (parent_p is None):
        return current_process()
    return None

def subscribe(topic, reqBlock=False, reqTimeout=0, ensureReply =True, replyTimeout = 10):
    """
    Subscribes to a given topic within the current execution context.

    Args:
        topic (str): The topic to subscribe to.
        reqBlock (bool, optional): Whether the subscription request should block until completion. Defaults to False.
        reqTimeout (int, optional): The timeout for the subscription request in seconds. Defaults to 0.
        ensureReply (bool, optional): Whether to ensure a reply is received for the subscription request. Defaults to True.
        replyTimeout (int, optional): The timeout for waiting for a reply in seconds. Defaults to 10.

    Raises:
        Exception: If the subscription is attempted from the main thread.
        Exception: If the current execution context is not a ManagedPubSubRunnable.

    Returns:
        Any: The result of the subscription request.
    """
    curr_t:ManagedPubSubRunnable = getCurrentExecutionContext()
    if curr_t is None:
        raise Exception("Cannot Subscribe from Main Thread")
    if not issubclass(type(curr_t), ManagedPubSubRunnable):
       raise Exception(f"Cannot Subscribe from Non Managed Runnable. Current Execution Context: {curr_t}")
    return curr_t.subscribe(topic, reqBlock=reqBlock, reqTimeout=reqTimeout, ensureReply=ensureReply, replyTimeout=replyTimeout)

def unsubscribe(topic, reqBlock=False, reqTimeout=0, ensureReply =True, replyTimeout = 10):
    """
    Unsubscribe from a given topic in the current execution context.

    Args:
        topic (str): The topic to unsubscribe from.
        reqBlock (bool, optional): Whether to block the request until it is processed. Defaults to False.
        reqTimeout (int, optional): The timeout for the request in seconds. Defaults to 0.
        ensureReply (bool, optional): Whether to ensure a reply is received. Defaults to True.
        replyTimeout (int, optional): The timeout for the reply in seconds. Defaults to 10.

    Raises:
        Exception: If called from the main thread.
        Exception: If the current execution context is not a ManagedPubSubRunnable.

    Returns:
        Any: The result of the unsubscribe operation from the current execution context.
    """
    curr_t:ManagedPubSubRunnable = getCurrentExecutionContext()
    if curr_t is None:
        raise Exception("Cannot Unsubscribe from Main Thread")
    if not issubclass(type(curr_t), ManagedPubSubRunnable):
       raise Exception(f"Cannot Unsubscribe from Non Managed Runnable. Current Execution Context: {curr_t}")
    return curr_t.unsubscribe(topic, reqBlock=reqBlock, reqTimeout=reqTimeout, ensureReply=ensureReply, replyTimeout=replyTimeout)

def publish(topic, payload, block=False, timeout=0.001)->bool:
    """
    Publish a message to a specified topic.

    Args:
        topic (str): The topic to which the message will be published.
        payload (Any): The message payload to be published.
        block (bool, optional): Whether to block until the message is published. Defaults to False.
        timeout (float, optional): The maximum time to wait for the message to be published. Defaults to 0.001 seconds.

    Returns:
        bool: True if the message was successfully published, False otherwise.

    Raises:
        Exception: If the function is called from the main thread.
        Exception: If the current execution context is not a ManagedPubSubRunnable.
    """
    curr_t:ManagedPubSubRunnable = getCurrentExecutionContext()
    if curr_t is None:
        raise Exception("Cannot Publish from Main Thread")
    if not issubclass(type(curr_t), ManagedPubSubRunnable):
       raise Exception(f"Cannot Publish from Non Managed Runnable. Current Execution Context: {curr_t}")
    return curr_t.publish(topic, payload, block=block, timeout=timeout)
    
def getMessages(block=True, timeout=1):
    """
    Retrieve messages from the current execution context.

    This function fetches messages from the current execution context, which must be an instance of `ManagedPubSubRunnable`.
    If the current execution context is the main thread or not a `ManagedPubSubRunnable`, an exception is raised.

    Args:
        block (bool, optional): Whether to block until a message is available. Defaults to False.
        timeout (int, optional): The maximum time to wait for a message if blocking. Defaults to 1 second.

    Returns:
        list: A list of messages retrieved from the current execution context. Can contain zero or more messages.

    Raises:
        Exception: If called from the main thread or from a non-managed runnable context.
    """
    curr_t:ManagedPubSubRunnable = getCurrentExecutionContext()
    if curr_t is None:
        raise Exception("Cannot Get Messages from Main Thread")
    if not issubclass(type(curr_t), ManagedPubSubRunnable):
       raise Exception(f"Cannot Get Messages from Non Managed Runnable. Current Execution Context: {curr_t}")
    return curr_t.getMessages(block=block, timeout=timeout)


#####################
# Pub Sub Classes   #
#####################
# The following classes are used to manage the pub-sub system, including runnables, messages, and the broker.
# These classes are meant to be used in the main process/thread to manage the pub-sub system.

class PubSubMsg():
    """
    PubSubMsg class provides static methods to create and handle publish-subscribe messages. This class is used so that changes to the message structure can be made in one place. As far as possible, the message structure should be kept simple and consistent across all nodes, and be implemented using pythons primitive or built-in types.

    For now, the messages themselves are tuples with the following structure:
    (message_type, sender, topic, payload)

    Attributes:
        _PUBLSH (int): Constant representing a publish message type.
        _SUBSCRIBE (int): Constant representing a subscribe message type.
        _UNSUBSCRIBE (int): Constant representing an unsubscribe message type.
    """
    _PUBLSH = 1
    _SUBSCRIBE = 2
    _UNSUBSCRIBE = 3

    ########################
    # Message Constructors #
    ########################
    @staticmethod
    def Message(topic, payload, name = None):
        """
        Creates a pubsub message. Used to transmit data between nodes.
        """
        sender = getCurrentExecutionContext().name if name is None else name
        return (PubSubMsg._PUBLSH, sender, topic, payload)
    
    @staticmethod
    def Subscribe(topic,name = None):
        """
        Creates a subscribe message. Used to notify the broker that the sender wants to subscribe to a topic.
        """
        sender = getCurrentExecutionContext().name if name is None else name
        return (PubSubMsg._SUBSCRIBE, sender, topic)
    
    @staticmethod
    def Unsubscribe(topic,name = None):
        """
        Creates an unsubscribe message. Used to notify the broker that the sender wants to unsubscribe from a topic.
        """
        sender = getCurrentExecutionContext().name if name is None else name
        return (PubSubMsg._UNSUBSCRIBE, sender, topic)
    
    ########################
    # Accessor Methods     #
    ########################
    @staticmethod
    def getMessageType(message):
        return message[0]

    @staticmethod
    def getSender(message):
        return message[1]
        
    @staticmethod
    def getPayload(message):
        if len(message) < 4:
            return None
        return message[3]
    
    @staticmethod
    def getTopic(message):
        return message[2]

    ########################
    # Utility Methods     #
    ########################
    @staticmethod
    def filterMessages(messages, topics):
        """
        Filters a list of messages based on the given topics.

        Args:
            messages (list): A list of messages to be filtered.
            topics (list): A list of topics to filter the messages by.

        Returns:
            list: A list of messages that belong to the specified topics.
        """
        return [x for x in messages if PubSubMsg.getTopic(x) in topics]

class ManagedPubSubRunnable():
    """
    ManagedPubSubRunnable is a base class that manages the publish-subscribe mechanism for a runnable task. This base class is not meant to be used directly, but should be subclassed to create context-specific runnables, such as threads or processes (See ManagedPubSubThread and ManagedPubSubProcess).

    Each ManagedPubSubRunnable has an input queue for receiving messages, an output queue for sending general messages.
    It also has a command input queue for sending and receiving lifecycle commands, which are used to subscribe and unsubscribe from topics.

    Runnables do not directly interact with each other. Instead, they communicate through the broker (See PubSubManager), which routes messages between them.

    Attributes:
        name (str): The name of the runnable.
        verbose (bool): If True, enables verbose logging.
        pubSubInput (Queue): The input queue for receiving messages.
        pubSubOutput (Queue): The output queue for sending messages.
        cmdInput (Queue): The command input queue.
        exitEvent (Event): The event to signal exit.
        startEvent (Event): The event to signal start.
    Methods:
        __init__(cmdInput: Queue = None, pubSubInput: Queue = None, pubSubOutput: Queue = None, exitEvent: Event = None, startEvent: Event = None, name = None, verbose = False):
            Initializes the ManagedPubSubRunnable with the provided parameters.
        setStartEvent(startEvent: Event):
            Sets the start event.
        publish(topic, payload, block=False, timeout=0.001) -> bool:
            Publishes a message to the specified topic.
        subscribe(topic, reqBlock=False, reqTimeout=0, ensureReply=False, replyTimeout=10) -> bool:
            Subscribes to the specified topic.
        unsubscribe(topic, reqBlock=False, reqTimeout=0, ensureReply=False, replyTimeout=10) -> bool:
            Unsubscribes from the specified topic.
        getMessages(block=False, timeout=1):
            Retrieves messages from the input queue.
        doExit():
            Sets the exit event.
        isExit() -> bool:
            Checks if the exit event is set.
    """

    def __init__(self, cmdInput:Queue=None, pubSubInput:Queue=None, pubSubOutput:Queue=None, exitEvent:Event=None, startEvent:Event=None, name=None, verbose=False):
        self.name = name
        self.verbose = verbose
        self.pubSubInput = pubSubInput
        self.pubSubOutput = pubSubOutput
        self.cmdInput = cmdInput
        self.exitEvent = exitEvent
        self.startEvent = None

    def setStartEvent(self, startEvent:Event):
        """
        Sets the start event for the PubSubManager. Make sure to set the start event before starting the runnable.

        Args:
            startEvent (Event): The event to be set as the start event.
        """
        self.startEvent = startEvent

    def publish(self, topic, payload, block=False, timeout=0.001) -> bool:
        """
        Publish a message to a specified topic.

        Args:
            topic (str): The topic to which the message will be published.
            payload (Any): The message payload to be published.
            block (bool, optional): Whether to block if the queue is full. Defaults to False.
            timeout (float, optional): The maximum time to block if `block` is True. Defaults to 0.001 seconds.

        Returns:
            bool: True if the message was successfully published, False otherwise.

        Raises:
            Exception: If an unexpected error occurs during publishing.

        """
        q: Queue = self.pubSubOutput
        name = self.name
        verbose = self.verbose
        try:
            q.put(PubSubMsg.Message(topic, payload), block=block, timeout=timeout)
            print(f"{name}: Published to channel: {topic}. Payload: {payload}") if verbose else None
            return True
        except Full:
            print(f"{name}: Broker's Buffer is full. Could not publish to channel: {topic}") if verbose else None
            return False
        except Exception as e:
            print(f"{name}: Error publishing to channel: {topic}. Error: {e}") if verbose else None
            return False

    def subscribe(self, topic, reqBlock=False, reqTimeout=0, ensureReply =False, replyTimeout = 10)->bool:
        """
        Subscribes to a given topic on the pub-sub system.
        Args:
            topic (str): The topic to subscribe to.
            reqBlock (bool, optional): Whether to block the request until it is processed. Defaults to False.
            reqTimeout (int, optional): The timeout for the request in seconds. Defaults to 0.
            ensureReply (bool, optional): Whether to wait for a reply from the broker. Defaults to False.
            replyTimeout (int, optional): The timeout for waiting for a reply in seconds. Defaults to 10.
        Returns:
            bool: True if the subscription was successful, False otherwise.
        Raises:
            Full: If the broker's buffer is full and the subscription message could not be sent.
            Exception: If there is an error sending the subscription message or receiving the reply.
        """
        sQ = self.pubSubOutput
        cmdQ = self.cmdInput
        name = self.name
        verbose = self.verbose
        try:
            r = sQ.put(PubSubMsg.Subscribe(topic), block=reqBlock, timeout=reqTimeout)
            print(f"{name}: Sent subscribe request to channel: {topic}") if verbose else None
            if not ensureReply:
                print(f"{name}: Subscribed (No Wait) to channel: {topic}") if verbose else None
                return True
        except Full:
            print(f"{name}: Broker's Buffer is full. Could not send Subscription Message for: {topic}") if verbose else None
            return False
        except Exception as e:
            print(f"{name}: Error sending Subscription Message: {topic}. Error: {e}") if verbose else None
            return False
        
        
        print(f"{name}: Waiting for Subscription Reply") if verbose else None
        try:
            reply = cmdQ.get(block=True, timeout=replyTimeout)
            if reply:
                print(f"{name}: Subscribed to channel: {topic}") if verbose else None
                return True
            else:
                print(f"{name}: Could not subscribe to channel: {topic}. Broker Rejected Request") if verbose else None
                return False
        except Empty:
            print(f"{name}: Could not subscribe to channel: {topic}. Request Timed Out.") if verbose else None
            return False
        except Exception as e:
            print(f"{name}: Error subscribing to channel: {topic}. Error: {e}") if verbose else None
            return False

    def unsubscribe(self, topic, reqBlock=False, reqTimeout=0, ensureReply =False, replyTimeout = 10)->bool:
        """
        Unsubscribe from a given topic.
        Parameters:
        topic (str): The topic to unsubscribe from.
        reqBlock (bool, optional): Whether to block the request until it is processed. Defaults to False.
        reqTimeout (int, optional): The timeout for the request in seconds. Defaults to 0.
        ensureReply (bool, optional): Whether to wait for a reply from the broker. Defaults to False.
        replyTimeout (int, optional): The timeout for the reply in seconds. Defaults to 10.
        Returns:
        bool: True if the unsubscription was successful, False otherwise.
        Raises:
        Full: If the broker's buffer is full and the unsubscription message could not be sent.
        Exception: If there is an error sending the unsubscription message or receiving the reply.
        """
        sQ = self.pubSubOutput
        cmdQ = self.cmdInput
        name = self.name
        verbose = self.verbose
        try:
            sQ.put( PubSubMsg.Unsubscribe(topic), block=reqBlock, timeout=reqTimeout)
            print(f"{name}: Sent unsubscribe request to channel: {topic}") if verbose else None
            if not ensureReply:
                print(f"Unsubscribed (No Wait) from channel: {topic}") if verbose else None
                return True
        except Full:
            print(f"{name}: Broker's Buffer is full. Could not send Unsubscription Message for: {topic}") if verbose else None
            return False
        except Exception as e:
            print(f"{name}: Error sending Unsubscription Message: {topic}. Error: {e}") if verbose else None
            return False
        
        print(f"{name}: Waiting for Unsubscribe Reply") if verbose else None
        try:
            reply = cmdQ.get(block=True, timeout=replyTimeout)
            if reply:
                print(f"{name}: Unsubscribed from channel: {topic}") if verbose else None
                return True
            else:
                print(f"{name}: Could not unsubscribe from channel: {topic}. Broker Rejected Request") if verbose else None
                return False
        except Empty:
            print(f"{name}: Could not unsubscribe from channel: {topic}. Request Timed Out.") if verbose else None
            return False
        except Exception as e:
            print(f"{name}: Error unsubscribing from channel: {topic}. Error: {e}") if verbose else None
            return False

    def getMessages(self, block=False, timeout=1):
        """
        Retrieve messages from the pubSubInput queue.
        Args:
            block (bool, optional): Whether to block if no messages are available. Defaults to False.
            timeout (int, optional): The maximum time to wait for a message if blocking. Defaults to 1 second.
        Returns:
            list: A list of messages retrieved from the queue.
        Raises:
            Exception: If an unexpected error occurs while retrieving messages.
        Notes:
            - If `verbose` is True, prints the approximate number of messages, and logs the number of messages received or any errors encountered.
            - If the queue is empty and `block` is False, returns an empty list.
        """
        q: Queue = self.pubSubInput
        name = self.name
        verbose = self.verbose

        messages = []
        approx_messages = max(q.qsize(),1)
        print(f"{name}: Approximate Messages: {approx_messages}. Block: {block} Timeout: {timeout} ")  if verbose else None

        try:
            for i in range(approx_messages):
                messages.append(q.get(block=block, timeout=timeout))
            print(f"{name}: Received {len(messages)} messages") if verbose else None
        except Empty:
            print(f"{name}: No messages available") if verbose else None
            return messages
        except Exception as e:
            print(f"{name}: Error getting messages: {e}") if verbose else None
        return messages
     
    def doExit(self):
        """
        Signals the exit event to terminate the process.

        This method sets the `exitEvent`, which is used to signal that the process
        should terminate. Call this when the application needs to
        perform a clean shutdown. In the `run` method, users should periodically check if the exit event is set and exit gracefully if it is.
        """
        self.exitEvent.set()

    def isExit(self):
        """
        Check if the exit event is set.

        This method checks if the `exitEvent` attribute is set. If `exitEvent` is `None`,
        it returns `False`. Otherwise, it returns the result of `exitEvent.is_set()`.

        Returns:
            bool: `True` if the exit event is set, `False` otherwise.
        """
        if self.exitEvent is None:
            return False
        return self.exitEvent.is_set()
    
class ManagedPubSubProcess(Process, ManagedPubSubRunnable):
    """
        A managed process that integrates pub/sub functionality with multiprocessing.
        This class extends both `Process` and `ManagedPubSubRunnable` to provide a
        multiprocessing process with pub/sub capabilities.

        The function to be run by the process should be passed as the `target` argument, and the arguments for the function should be passed as the `args` and `kwargs` arguments.

        The runnable process can be started with the `start` method, which is implemented in the `Process` superclass. If the `startEvent` attribute is set, the process will wait for the event to be set before running the target function.

        These processes should not be started directly. Instead, users should use the `PubSubManager` class to manage the creation, starting, and stopping of these processes.
        
        Methods:
            __init__(group=None, target=None, name=None, args=(), kwargs={}, daemon=True, 
                     cmdInput=None, pubSubInput=None, pubSubOutput=None, exitEvent=None, verbose=False):
                Initializes a new instance of the ManagedPubSubProcess class.
            run():
                Runs the process, waiting for the start event if it is set, then calls the superclass's `run` method.
    """

    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, daemon=True, 
                 cmdInput:Queue=None, pubSubInput:Queue=None, pubSubOutput:Queue=None, exitEvent:Event=None,
                 verbose=False):
        """
        Initialize a new instance of the class.

        Args:
            group (Optional[Any]): The group argument is not currently used.
            target (Optional[Callable]): The callable object to be invoked by the run() method.
            name (Optional[str]): The process name.
            args (tuple): The argument tuple for the target invocation.
            kwargs (dict): A dictionary of keyword arguments for the target invocation.
            daemon (bool): Whether this process is a daemon process.
            cmdInput (Queue, optional): Queue for command input.
            pubSubInput (Queue, optional): Queue for pub/sub input.
            pubSubOutput (Queue, optional): Queue for pub/sub output.
            exitEvent (Event, optional): Event to signal process exit.
            verbose (bool): If True, enables verbose logging.

        """
        Process.__init__(self, group=group, target=target, name=name, args=args, kwargs=kwargs, daemon=daemon)
        ManagedPubSubRunnable.__init__(self, cmdInput=cmdInput, pubSubInput=pubSubInput, pubSubOutput=pubSubOutput, exitEvent=exitEvent, name=name, verbose=verbose)
    
    def run(self):
        """
        Runs the thread, waiting for the start event if it is set. Thereafer it calls the superclass's `run` method. In this case, we call Python's `Process` class's `run` method.

        If `startEvent` is not None, this method will wait for the event to be set
        before proceeding to call the `run` method of the superclass.

        Returns:
            The result of the superclass's `run` method.
        """
        if self.startEvent is not None:
            self.startEvent.wait()
        return super().run()

class ManagedPubSubThread(Thread, ManagedPubSubRunnable):
    """
    A managed thread that integrates pub/sub functionality with threading.
    This class extends both `Thread` and `ManagedPubSubRunnable` to provide a
    threading thread with pub/sub capabilities.

    The function to be run by the thread should be passed as the `target` argument, and the arguments for the function should be passed as the `args` and `kwargs` arguments.

    The runnable thread can be started with the `start` method, which is implemented in the `Thread` superclass. If the `startEvent` attribute is set, the thread will wait for the event to be set before running the target function.

    These threads should not be started directly. Instead, users should use the `PubSubManager` class to manage the creation, starting, and stopping of these threads.
    
    Methods:
        __init__(group=None, target=None, name=None, args=(), kwargs={}, daemon=True, 
                 cmdInput=None, pubSubInput=None, pubSubOutput=None, exitEvent=None, verbose=False):
            Initializes a new instance of the ManagedPubSubThread class.
        run():
            Runs the thread, waiting for the start event if it is set, then calls the superclass's `run` method.
    """

    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, daemon=True, 
                 cmdInput:Queue=None, pubSubInput:Queue=None, pubSubOutput:Queue=None, exitEvent:Event=None,
                 verbose=False):
        Thread.__init__(self, group=group, target=target, name=name, args=args, kwargs=kwargs, daemon=daemon)
        ManagedPubSubRunnable.__init__(self, cmdInput=cmdInput, pubSubInput=pubSubInput, pubSubOutput=pubSubOutput, exitEvent=exitEvent, name=name, verbose=verbose)
    
    def run(self):
        if self.startEvent is not None:
            self.startEvent.wait()
        return super().run()
    
class PubSubManager():
    """
    PubSubManager is a class that manages a publish-subscribe messaging system with support for both threads and processes.

    The PubSubManager class provides methods to add and manage threads and processes, start and stop them, and handle message routing between them. This manager is meant to be used with python's in-built "with" statement to ensure proper cleanup and termination of threads and processes. 

    The pubsub manager consits of mainly two components:
    1. Broker: The broker is responsible for routing messages between threads and processes. It maintains a tree structure of topics and subscribers, and routes messages to the appropriate subscribers based on the topic. All nodes (threads and processes) communicate with the broker to subscribe, unsubscribe, and publish messages. The broker is implemented as a thread that runs in the main process.

    2. Process Relay: The process relay is responsible for relaying messages from processes to threads. The underlying communication methods for threads and processes are different, so the process relay acts as a bridge between them. It receives messages from processes and relays them to the broker, which can then route them to the appropriate subscribers. The process relay is implemented as a thread that runs in the main process.

    Refer to the provided alex_example_pubsub.py for an example of how to use the PubSubManager class.

    Attributes:
        verbose (bool): If True, enables verbose logging.
        channels (dict): A dictionary to store channels and their subscribers.
        executables (dict): A dictionary to store managed threads and processes.
        threadMessageQueue (Queue): A queue for thread messages.
        processMessageQueue (mQueue): A queue for process messages.
        threadExitEvent (Event): An event to signal thread exit.
        processExitEvent (mEvent): An event to signal process exit.
        _brokerThread (Thread): A thread for the broker task.
        _processRelayThread (Thread): A thread for the process relay task.
        _startBarrier (Barrier): A barrier to synchronize the start of threads.
    Methods:
        __init__(self, verbose=False):
            Initializes the PubSubManager with optional verbose logging.
        add_process(self, group=None, target=None, name=None, args=(), kwargs={}, daemon=True, verbose=False):
            Adds a managed process to the PubSubManager.
        add_thread(self, group=None, target=None, name=None, args=(), kwargs={}, daemon=True, verbose=False):
            Adds a managed thread to the PubSubManager.
        start_all(self):
            Starts all managed threads and processes.
        exit_all(self):
            Signals all managed threads and processes to exit.
        join_all(self):
            Waits for all managed threads and processes to complete.
        parseTopic(self, topic: str):
            Parses a topic string into its components.
        getTopicTargets(self, topic):
            Retrieves the targets (subscribers) for a given topic.
        addSubscriber(self, topic, subscriber):
            Adds a subscriber to a given topic.
        removeSubscriber(self, topic, subscriber):
            Removes a subscriber from a given topic.
        _processRelayThreadTask(self, verbose=False):
            The task for the process relay thread.
        _brokerThreadTask(self, verbose=False):
            The task for the broker thread.
        __enter__(self):
            Starts the broker and process relay threads and waits for them to be ready.
        __exit__(self, exc_type, exc_value, traceback):
            Signals all threads and processes to exit and waits for them to complete.
    """

    def __init__(self, verbose=False):
        """
        Initializes the PubSubManager.
        Args:
            verbose (bool): If True, enables verbose logging. Defaults to False.
        Attributes:
            verbose (bool): Stores the verbosity setting.
            channels (dict): A dictionary containing channels and subscribers.
            executables (dict): A dictionary to store executable tasks.
            threadMessageQueue (Queue): A queue for thread messages.
            processMessageQueue (mQueue): A queue for process messages.
            threadExitEvent (Event): An event to signal thread exit.
            processExitEvent (mEvent): An event to signal process exit.
            _brokerThread (Thread): A thread for handling broker tasks.
            _processRelayThread (Thread): A thread for handling process relay tasks.
            _startBarrier (Barrier): A barrier to synchronize the broker and process relay threads.
        """
        self.verbose = verbose

        self.channels = {"channels": {}, "subscribers": set()}
        self.executables = {}

        self.threadMessageQueue = Queue()
        self.processMessageQueue = mQueue()

        self.threadExitEvent = Event()
        self.processExitEvent = mEvent()

        self._brokerThread = Thread(target=self._brokerThreadTask, kwargs={"verbose":verbose})
        self._processRelayThread = Thread(target=self._processRelayThreadTask, kwargs={"verbose":verbose})
        self._startBarrier = Barrier(3)


    ##############################
    # Runnable Lifecycle Methods #
    ##############################
    # These methods are used by the user to manage the lifecycle of threads and processes.
    # They should only be called from the main thread.
    # They should only be called within a `with` statement to ensure proper cleanup. (See alex_example_pubsub.py for an example)

    def add_process(self, group=None, target=None, name=None, args=(), kwargs={}, daemon=True, verbose=False):
        """
        Adds a new managed process to the pub-sub manager.
        Parameters:
        group (Optional[Any]): The process group. Defaults to None.
        target (Optional[Callable]): The target function to be executed by the process. Defaults to None.
        name (Optional[str]): The name of the process. If not provided, a default name is generated. Defaults to None.
        args (tuple): The arguments to pass to the target function. Defaults to ().
        kwargs (dict): The keyword arguments to pass to the target function. Defaults to {}.
        daemon (bool): Whether the process should be a daemon process. Defaults to True.
        verbose (bool): Whether to enable verbose logging for the process. Defaults to False.
        Returns:
        bool: True if the process was added successfully.
        """
        inQ = mQueue()
        cmdQ = mQueue(maxsize=1)
        outQ = self.processMessageQueue
        process_count = len(self.executables)

        name = name if name else f"Executable-{process_count+1}-p"
        p = ManagedPubSubProcess(group=group, target=target, name=name, args=args, kwargs=kwargs, daemon=daemon,
                                    cmdInput=cmdQ, pubSubInput=inQ, pubSubOutput=outQ, exitEvent=self.processExitEvent, verbose=verbose)
        self.executables[name] = {"executable": p, "input": inQ, "output": outQ , "command": cmdQ}
        return True
    
    def add_thread(self, group=None, target=None, name=None, args=(), kwargs={}, daemon=True, verbose=False):
        """
        Adds a new thread to the manager.

        Parameters:
        group (None or str): The group to which the thread belongs. Default is None.
        target (callable): The callable object to be invoked by the thread's run() method. Default is None.
        name (str): The thread name. If None, a default name is generated. Default is None.
        args (tuple): The argument tuple for the target invocation. Default is ().
        kwargs (dict): A dictionary of keyword arguments for the target invocation. Default is {}.
        daemon (bool): Whether the thread is a daemon thread. Default is True.
        verbose (bool): If True, enables verbose output. Default is False.

        Returns:
        bool: True if the thread was added successfully.
        """
        inQ = Queue()
        cmdQ = Queue(maxsize=1)
        outQ = self.threadMessageQueue  
        name = name if name else f"Executable-{len(self.executables)+1}-t"
        t = ManagedPubSubThread(group=group, target=target, name=name, args=args, kwargs=kwargs, daemon=daemon,
                                    cmdInput=cmdQ,pubSubInput=inQ, pubSubOutput=outQ, exitEvent=self.threadExitEvent, verbose=verbose)
        self.executables[name] = {"executable": t, "input": inQ, "output": outQ, "command": cmdQ}
        return True

    def start_all(self):
        """
        Starts all executables managed by this instance.
        It initializes events for threads and processes, sets the start event for
        each executable, and then starts them.
        Attributes:
            start_thread_event (Event): Event to signal the start of thread-based executables.
            start_process_event (mEvent): Event to signal the start of process-based executables.
        Raises:
            Any exceptions raised by the `start` method of the executables.

        Note:
            The main process should not hold any system locks, as they will be inherited by child processes and may cause deadlock. System locks include threading locks, semaphores, and queues, but also other resources like file descriptors, which include standard input/output/error. This means that the main process should not be reading from standard input or writing to standard output/error (i.e., using print) while spawning child processes.
        """
        # Need to make sure that main process does not hold any system locks
        # They will be inherited by child processes, and might cause deadlock
        start_thread_event = Event()
        start_process_event = mEvent()

        for ex in self.executables:
            start_event = start_thread_event if isinstance(self.executables[ex]["executable"], Thread) else start_process_event
            self.executables[ex]["executable"].setStartEvent(start_event)
            self.executables[ex]["executable"].start()

        start_process_event.set()
        start_thread_event.set()

    def exit_all(self):
        """
        Signals all threads and processes to exit by setting the respective exit events.

        This method checks if the thread and process exit events are not already set,
        and if not, sets them to signal the threads and processes to terminate. 
        
        This does not actually terminate the threads and processes, but signals them to exit gracefully.
        Therefore, all threads and processes should periodically check if the exit event is set and exit gracefully

        Returns:
            None
        """
        if not self.threadExitEvent.is_set():
            self.threadExitEvent.set()
        if not self.processExitEvent.is_set():
            self.processExitEvent.set()

    def join_all(self):
        """
        Waits for all executable threads to complete.

        This method iterates over all executables in the `self.executables` dictionary
        and calls the `join` method on each executable thread, ensuring that the main
        program waits for these threads/processes to finish execution before proceeding.
        """

        thread_executables = [ex for ex in self.executables if isinstance(self.executables[ex]["executable"], ManagedPubSubThread)]
        process_executables = [ex for ex in self.executables if isinstance(self.executables[ex]["executable"], ManagedPubSubProcess)]

        order = (thread_executables, process_executables)
        for ex_list in order:
            for idx, ex in enumerate(ex_list):
                self.executables[ex]["executable"].join()
                q = self.executables[ex]["input"]
                while not q.empty():
                    q.get()
                q = self.executables[ex]["output"]
                while not q.empty():
                    q.get()
                q = self.executables[ex]["command"]
                while not q.empty():
                    q.get()
        return True

    #####################
    # Topic Management  #
    #####################
    # These methods are used by the broker to manage topics and subscribers in the pub-sub system.
    # Runnables (threads and processes) should NOT call these methods to subscribe, unsubscribe, and publish messages.

    def parseTopic(self,topic:str):
        """
        Parses a topic string into its components.

        Args:
            topic (str): The topic string to be parsed, with components separated by '/'.

        Returns:
            list: A list of strings, each representing a component of the topic.
        """
        return topic.split("/")

    def getTopicTargets(self, topic):
        """
        Retrieve the set of subscribers for a given topic.

        This method traverses the hierarchical structure of topics and subtopics
        to gather all subscribers associated with the specified topic and its subtopics.

        Args:
            topic (str): The topic string to retrieve subscribers for.

        Returns:
            set: A set of subscribers associated with the given topic and its subtopics.
        """
        targets = set(self.channels["subscribers"])
        parts = self.parseTopic(topic)
        current_topic = self.channels["channels"]
        for subtopic in parts:
            if subtopic in current_topic:
                new_topic = current_topic[subtopic]
                targets = targets.union(new_topic["subscribers"])
                current_topic = new_topic["channels"]
            else:
                break
        return targets

    def addSubscriber(self, topic, subscriber):
        """
        Adds a subscriber to a specific topic.
        This method parses the given topic into its subtopics, navigates through the
        channel hierarchy, and adds the subscriber to the set of subscribers for the
        final subtopic.
        Args:
            topic (str): The topic to which the subscriber should be added. This can be a
                         hierarchical topic separated by delimiters.
            subscriber (Any): The subscriber to be added to the topic. This can be any
                              object that represents a subscriber.
        Returns:
            bool: True if the subscriber was successfully added.

        Example Topics:
            - "a/b/c": A hierarchical topic with three subtopics. The subscriber will be added to the "c" subtopic.
            - "a": A single-level topic. The subscriber will be added to the "a" topic, and receive all messages published to it and its subtopics.
            - "": The root topic. The subscriber will be added to the root topic and receive all messages published to any topic.

        """
        parts = self.parseTopic(topic)
        current_topic = self.channels
        print(f"Adding {subscriber} to {parts} subscribers\n{self.channels}") if self.verbose else None
        if parts != [""]:
            for subtopic in parts:
                current_topic = current_topic["channels"]
                if subtopic not in current_topic:
                    current_topic[subtopic] = {"channels": {}, "subscribers": set()}
                current_topic = current_topic[subtopic]

        current_topic["subscribers"].add(subscriber)
        print(f"Added {subscriber} to {topic} subscribers\n{self.channels}") if self.verbose else None
        return True

    def removeSubscriber(self, topic, subscriber):
        """
        Removes a subscriber from a given topic.

        Args:
            topic (str): The topic from which the subscriber should be removed.
            subscriber (Any): The subscriber to be removed.

        Returns:
            bool: True if the subscriber was successfully removed, False otherwise.

        Raises:
            ValueError: If the subscriber is not found in the topic's subscriber list.
        """
        parts = self.parseTopic(topic)
        current_topic = self.channels
        if parts != [""]:
            for subtopic in parts:
                current_topic = current_topic["channels"]
                if subtopic not in current_topic:
                    return False
                current_topic = current_topic[subtopic]
        current_topic["subscribers"].remove(subscriber)
        print(f"Removed {subscriber} to {topic} subscribers\n{self.channels}") if self.verbose else None
        return True

    def getAllSubscribedTopics(self, subscriber):
        """
        Navigates the entire channel hierarchy to find all topics to which a subscriber is subscribed.

        Args:
            subscriber (Any): The subscriber for which to retrieve topics.

        Returns:
            list: A list of topics to which the subscriber is subscribed.
        """
        topics = set()
        def _getTopics(subscriber, current_topic, topic):
            if subscriber in current_topic["subscribers"]:
                topics.add(topic)
            for subtopic in current_topic["channels"]:
                _getTopics(subscriber, current_topic["channels"][subtopic], f"{topic}/{subtopic}")
        _getTopics(subscriber, self.channels, "")
        return topics


    #####################
    # Manager Tasks     #
    #####################
    # These methods define the broker and process relay threads to handle message routing and relay.
    # They should not be called directly by the user.

    def _processRelayThreadTask(self, verbose=False):
        """
        Handles the relay of messages between the process message queue and the thread message queue.
        This method runs in a separate thread and continuously checks for messages in the process message queue.
        If a message is found, it attempts to put it into the thread message queue. The method exits when either
        the `threadExitEvent` or `processExitEvent` is set.
        Args:
            verbose (bool): If True, prints detailed debug information. Defaults to False.
        Raises:
            Exception: If an unexpected error occurs while getting a message from the process queue or putting a message into the thread queue.
        """
        print("Process Relay Thread: Started") if verbose else None
        self._startBarrier.wait(timeout=10)
        while not (self.threadExitEvent.is_set() or self.processExitEvent.is_set()):
            try:
                result = self.processMessageQueue.get(block=True, timeout=1)
            except Empty:
                continue
            except Exception as e:
                print(f"Error getting message from Process Queue: {e}") if verbose else None
                continue

            try:
                self.threadMessageQueue.put(result, block=True, timeout=1)  
            except Full:
                print("Thread's Buffer is full. Could not relay message") if verbose else None
                continue
            except Exception as e:
                print(f"Error relaying message to Thread Queue: {e}") if verbose else None
                continue
        print("Process Relay Thread: Exited")

    def _brokerThreadTask(self,verbose=False):
        """
        The broker thread task that handles publishing and subscribing messages in a pub-sub system.
        The broker task continuously processes messages from the thread message queue until either the threadExitEvent or processExitEvent is set.
        It processes messages based on their type (PUBLISH, SUBSCRIBE, UNSUBSCRIBE) and updates the channel hierarchy and subscriber lists accordingly.
        Args:
            verbose (bool): If True, prints detailed debug information. Default is False.
        Raises:
            Exception: If there are errors during message processing or queue operations.
        """
        print("Broker Thread: Started") if verbose else None
        self._startBarrier.wait(timeout=10)
        while not (self.threadExitEvent.is_set() or self.processExitEvent.is_set()):
            try:
                message = self.threadMessageQueue.get(block=True, timeout=1)
            except Empty:
                continue
            except Exception as e:
                print(f"Error getting message from Thread Queue: {e}") if verbose else None
                continue
            print(f"Broker: Received Message: {message} from {current_thread().name}") if verbose else None
            messageType = PubSubMsg.getMessageType(message)
            if messageType == PubSubMsg._PUBLSH:
                topic = PubSubMsg.getTopic(message)
                targets = self.getTopicTargets(topic)
                print(f"Broker: Publishing {message} to {targets} targets") if verbose else None
                for target in targets:
                    try:
                        # Check if the target is alive
                        target_is_alive = self.executables[target]["executable"].is_alive()
                        if not target_is_alive:
                            print(f"Broker: Could not send message to {target}. Target is not alive") if verbose else None
                            continue

                        # Publish to the target
                        target_message_queue = self.executables[target]["input"]
                        target_message_queue.put(message, block=False)
                        print(f"Broker: Published to {target}") if verbose else None
                    except Full:
                        print(f"Broker: {target}'s Buffer is full. Could not publish to channel: {topic}") if verbose else None
                    except Exception as e:
                        print(f"Broker: Error publishing to {target}. Error: {e}") if verbose else None

            elif messageType == PubSubMsg._SUBSCRIBE:
                try:
                    topic = PubSubMsg.getTopic(message)
                    subscriber = PubSubMsg.getSender(message)
                    subscriber_cmdQ:Queue = self.executables[subscriber]["command"]
                    result = self.addSubscriber(topic, subscriber)
                except Exception as e:
                    print(f"Broker: Error subscribing to channel: {topic}. Error: {e}") if verbose else None

                try:
                    subscriber_cmdQ.put(result, block=False)
                except Full:
                    print(f"Broker: Thread's Buffer is full. Could not send Subscription Reply for: {topic}") if verbose else None
                except Exception as e:
                    print(f"Broker: Error sending Subscription Reply: {topic}. Error: {e}") if verbose else None

            elif messageType == PubSubMsg._UNSUBSCRIBE:
                try:
                    topic = PubSubMsg.getTopic(message)
                    subscriber = PubSubMsg.getSender(message)
                    subscriber_cmdQ:Queue = self.executables[subscriber]["command"]
                    result = self.removeSubscriber(topic, subscriber)
                except Exception as e:
                    print(f"Broker: Error unsubscribing from channel: {topic}. Error: {e}") if verbose else None

                try:
                    subscriber_cmdQ.put(result, block=False)
                except Full:
                    print(f"Broker: Thread's Buffer is full. Could not send Unsubscription Reply for: {topic}") if verbose else None
                except Exception as e:
                    print(f"Broker: Error sending Unsubscription Reply: {topic}. Error: {e}") if verbose else None
            
        # Cleaning up
        if not self.threadExitEvent.is_set():
            self.threadExitEvent.set()
        if not self.processExitEvent.is_set():
            self.processExitEvent.set()
        print("Broker Thread: Exited")



    #####################
    # Manager Lifecycle #
    #####################
    # These methods are used to manage the lifecycle of the PubSubManager.
    # They implement pythons context manager protocol (i.e., using the 'with' statement)
    # __enter__ is called when the 'with' statement is used, and __exit__ is called when the code block exits.
    def __enter__(self):
        """
        Enter the runtime context (i.e., when the 'with' statement is used).

        This method is called when the 'with' statement is used. It starts the broker and process relay threads,
        and waits for them to be ready. If the threads do not start within the specified timeout, a 
        BrokenBarrierError is caught and a message is printed.

        Returns:
            self: The instance of the class.
        """
        self._brokerThread.start()
        self._processRelayThread.start()
        try:
            self._startBarrier.wait(timeout=20)
        except BrokenBarrierError as e:
            print(f"Wating for Broker and Process Relay Threads to start timed out")
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        """
        Handles the cleanup and shutdown of threads when exiting the context (i.e., when the 'with' statement ends).

        This method is called when the context manager exits. It ensures that
        all threads are properly stopped and joined.

        Args:
            exc_type (type): The exception type, if an exception was raised.
            exc_value (Exception): The exception instance, if an exception was raised.
            traceback (traceback): The traceback object, if an exception was raised.

        Returns:
            bool: Always returns True to suppress any exceptions.
        """
        self.exit_all()
        self.join_all()
        self._brokerThread.join()
        self._processRelayThread.join()
        return True   

                
        

