import time,sys
from threading import Barrier
from multiprocessing import Barrier as mBarrier

from pubsub.pub_sub_manager import PubSubManager, ManagedPubSubRunnable, ManagedPubSubProcess, ManagedPubSubThread
from pubsub.pub_sub_manager import PubSubMsg, publish, subscribe, unsubscribe, getMessages, getCurrentExecutionContext

import random


def echo_publisher_thread( topic:str,  print_info:bool=False):
    # This is the context (i.e., the node) that is running this code
    # You can get the current context by calling getCurrentExecutionContext() anywhere in the code
    # It will return the context of the node that is running the code
    # And allow you to interact with the PubSub system
    ctx:ManagedPubSubRunnable = getCurrentExecutionContext()

    # Lets see what ctx can tell use about the node and the pubsub system
    informationString = f"=====================================\n"
    informationString += "Node Information:\n"
    
    # Each node has a name, either set by the user or automatically generated
    name = ctx.name
    informationString += f"Name: {name}\n"

    # Each node is either a process or a thread
    # This can be used to determine if the node is running in a separate process or thread
    # ManagedPubSubProcess inhetits from both multiprocessing.Process and ManagedPubSubRunnable
    # ManagedPubSubThread inhetits from both threading.Thread and ManagedPubSubRunnable 
    isProcess = isinstance(ctx, ManagedPubSubProcess)
    isThread = isinstance(ctx, ManagedPubSubThread)
    informationString += f"Node Type: {'Process' if isProcess else ''}{'Thread' if isThread else ''}\n"

    # There is an exit flag that can be checked to see if the node should exit
    # This Flag is part of PubSubManager
    # Once set, it will be set for all nodes
    isExit = ctx.isExit()
    informationString += f"Exit Flag Set?: {isExit}\n"
    informationString += f"=====================================\n"
    
    print(informationString) if print_info else None

    
    try:
        while not ctx.isExit():
            user_input = input("Enter a message to echo: \n") 
            publish(topic= topic, payload=user_input)
            time.sleep(1)
    except KeyboardInterrupt:
        ctx.doExit()

    print("Publisher Node Exiting")

def echo_subscriber_thread(topic:str):
    ctx:ManagedPubSubRunnable = getCurrentExecutionContext()

    # Subscribe to the topic
    subscribe(topic=topic, ensureReply=True, replyTimeout=1)

    try:
        while not ctx.isExit():
            messages = getMessages(block=True, timeout=1)
            if messages:
                for m in messages:
                    sender = PubSubMsg.getSender(m) # The name of the node that sent the message
                    topic = PubSubMsg.getTopic(m)  # The topic the message was sent on
                    payload = PubSubMsg.getPayload(m) # The payload of the message
                    print(f"Echo: {payload} from {sender} on {topic}")
    except KeyboardInterrupt:
        ctx.doExit()

    print("Subscriber Node Exiting")

def spooky_publisher_thread(hauntInterval= (1,5)):
    ctx:ManagedPubSubRunnable = getCurrentExecutionContext()
    spookyname=  ctx.name

    subscribe(topic=f"Batarang/{spookyname}", ensureReply=True, replyTimeout=1)

    spookyNoises = [
        "Boo!",
        "Huehuehuehue!",
        "Nom nom nom!",
        "Beware!",
        "SpoOOOooOOOky!",
    ]

    try:
        while not ctx.isExit():
            time.sleep(random.randint(*hauntInterval))

            m = getMessages(block=True, timeout=1)
            if m:
                # Got hit by a batarang
                print(f"{spookyname} Got hit by a batarang! Going to Hide!")
                time.sleep(10)
                continue
            
            publish(topic=f"chat/spooky/{spookyname}", payload=random.choice(spookyNoises))


    except KeyboardInterrupt:
        ctx.doExit()

    print(f" {spookyname} Exiting")

def batman_thread():
    ctx:ManagedPubSubRunnable = getCurrentExecutionContext()
    batmanname=  ctx.name

    subscribe(topic=f"chat/spooky", ensureReply=True, replyTimeout=1)

    ghostIsBeingWatched = []

    try:
        while not ctx.isExit():
            messages = getMessages(block=True, timeout=1)
            if messages:
                for m in messages:
                    sender = PubSubMsg.getSender(m) # The name of the node that sent the message

                    if not (sender in ghostIsBeingWatched):
                        ghostIsBeingWatched.append(sender)
                        print(f"Batman: {sender} is being watched!")
                    if len(ghostIsBeingWatched) == 5:
                        print("Batman: I have to stop them!")
                        for ghost in ghostIsBeingWatched:
                            publish(topic=f"Batarang/{ghost}", payload="Batarang!")
                        print("Batman: Batarangs Away! Taking a break!")
                        time.sleep(10)
                        ghostIsBeingWatched = []

    except KeyboardInterrupt:
        ctx.doExit()

    print("Batman Exiting")

def main():

    # PubSubManager is used with python's "with" statement to ensure startup and shutdown of the pubsub system
    with PubSubManager() as mgr:

        # Input Node (Publisher Node)
        mgr.add_thread(
            target=echo_publisher_thread,   # Function to run
            name = "InputMan",              # Name of the node
            args=("chat/userInput", ),      # Arguments to pass to the function
            kwargs={"print_info":True}      # Keyword arguments to pass to the function
            )

        # Echo Node (Subscriber Node)
        # mgr.add_thread(target=echo_subscriber_thread, name="EchoMan", args=("chat",))

        # Spooky Node (Subscriber Node)
        number_of_spooky_nodes = 0
        for i in range(number_of_spooky_nodes):
            spookyname = f"SpookyGhost{i}"
            twoSpookyNumbers = random.randint(3, 20), random.randint(3, 20)
            twoSpookyNumbers = (min(twoSpookyNumbers), max(twoSpookyNumbers))

            mgr.add_thread(
                target=spooky_publisher_thread,
                name=spookyname,
                kwargs={"hauntInterval":twoSpookyNumbers}
            ) 


        # Batman Node (Combo Node)
        # mgr.add_thread(target=batman_thread, name="Batman")

        # Start all Threads
        mgr.start_all()

        # Wait for all threads to finish
        mgr.join_all()


if __name__ == "__main__":
    main()