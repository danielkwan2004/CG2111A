#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"
#include <sys/time.h>


//check if this is allowed
#include <termios.h>
#include <fcntl.h>
#define CMD_COOLDOWN_MS 200
static struct termios oldt, newt;
static int oldf;
//end of allowed check


#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B9600

int exitFlag = 0;
sem_t _xmitSema;


void sendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len);
}

void sendCommandWithCooldown(TPacket *cmd) 
{
    static struct timeval lastSendTime = {0,0};
    struct timeval now;
    gettimeofday(&now, NULL);
    long diffMS = (now.tv_sec - lastSendTime.tv_sec) * 1000L + (now.tv_usec - lastSendTime.tv_usec) / 1000L;
    if (diffMS >= CMD_COOLDOWN_MS) {
	sendPacket(cmd);
	lastSendTime = now;
    }    
    else {
    	printf("don't press so fast...");
    }
}


//check if this is allowed
void enableRawMode()
{
    tcgetattr(STDIN_FILENO, &oldt);           // get current terminal settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);         // disable canonical mode & echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // set new terminal settings

    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
}

void disableRawMode()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // restore old terminal settings
    fcntl(STDIN_FILENO, F_SETFL, oldf);
}

int keyPressed()
{
    // Returns a character if pressed, else 0 if nothing is there
    unsigned char ch;
    if(read(STDIN_FILENO, &ch, 1) == 1)
        return ch;
    return 0;
}

//end of allowed check



void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			printf("ERROR: Bad Magic Number\n");
			break;

		case PACKET_CHECKSUM_BAD:
			printf("ERROR: Bad checksum\n");
			break;

		default:
			printf("ERROR: UNKNOWN ERROR\n");
	}
}

void handleStatus(TPacket *packet)
{
	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", packet->params[0]);
	printf("Right Forward Ticks:\t\t%d\n", packet->params[1]);
	printf("Left Reverse Ticks:\t\t%d\n", packet->params[2]);
	printf("Right Reverse Ticks:\t\t%d\n", packet->params[3]);
	printf("Left Forward Ticks Turns:\t%d\n", packet->params[4]);
	printf("Right Forward Ticks Turns:\t%d\n", packet->params[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", packet->params[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", packet->params[7]);
	printf("Forward Distance:\t\t%d\n", packet->params[8]);
	printf("Reverse Distance:\t\t%d\n", packet->params[9]);
	printf("\n---------------------------------------\n\n");
}

void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			printf("Command OK\n");
		break;

		case RESP_STATUS:
			handleStatus(packet);
		break;

		default:
			printf("Arduino is confused\n");
	}
}

void handleErrorResponse(TPacket *packet)
{
	// The error code is returned in command
	switch(packet->command)
	{
		case RESP_BAD_PACKET:
			printf("Arduino received bad magic number\n");
		break;

		case RESP_BAD_CHECKSUM:
			printf("Arduino received bad checksum\n");
		break;

		case RESP_BAD_COMMAND:
			printf("Arduino received bad command\n");
		break;

		case RESP_BAD_RESPONSE:
			printf("Arduino received unexpected response\n");
		break;

		default:
			printf("Arduino reports a weird error\n");
	}
}

void handleMessage(TPacket *packet)
{
	printf("Message from Alex: %s\n", packet->data);
}

void handlePacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
				// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
				handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
				handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
				handleMessage(packet);
			break;
		case PACKET_TYPE_COLOR:
			printf("Color sensor reading: %s\n", packet->data);
			break;
	}
}


void *receiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handlePacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					printf("PACKET ERROR\n");
					handleError(result);
				}
		}
	}
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

void getParams(TPacket *commandPacket)
{
	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	scanf("%d %d", &commandPacket->params[0], &commandPacket->params[1]);
	flushInput();
}
/*
void sendCommand(char command)
{
    TPacket commandPacket;
    commandPacket.packetType = PACKET_TYPE_COMMAND;

    // We'll set a large distance/angle so it keeps moving
    // until we tell it to stop
    int distanceOrAngle = 9999;

    switch(command)
    {
        // W = forward
        case 'w':
	case 'W':
            commandPacket.command = COMMAND_FORWARD;
            commandPacket.params[0] = distanceOrAngle; // "distance" just a big number
            commandPacket.params[1] = 40;              // min 50% power
            sendPacket(&commandPacket);
            break;

        // S = backward
        case 's':
	case 'S':
            commandPacket.command = COMMAND_REVERSE;
            commandPacket.params[0] = distanceOrAngle;
            commandPacket.params[1] = 40;              // min 50% power
            sendPacket(&commandPacket);
            break;

        // A = turn left
        case 'a':
	case 'A':
            commandPacket.command = COMMAND_TURN_LEFT;
            commandPacket.params[0] = distanceOrAngle; // treat as "angle"
            commandPacket.params[1] = 75;              // min 75% power
            sendPacket(&commandPacket);
            break;

        // D = turn right
        case 'd':
	case 'D':
            commandPacket.command = COMMAND_TURN_RIGHT;
            commandPacket.params[0] = distanceOrAngle;
            commandPacket.params[1] = 75;              // min 85% power
            sendPacket(&commandPacket);
            break;

        // Q = quit
        case 'q':
	case 'Q':
            exitFlag = 1;
            break;

	// Z = open claw
	case 'z':
	case 'Z':
	    commandPacket.command = COMMAND_OPEN_CLAW;
	    sendPacket(&commandPacket);
	    break;
	
	// C = close claw
	case 'C':
	case 'c':
	    commandPacket.command = COMMAND_CLOSE_CLAW; 
	    sendPacket(&commandPacket);
	    break;

        // Anything else => stop
        default:
            commandPacket.command = COMMAND_STOP;
            commandPacket.params[0] = 0;
            commandPacket.params[1] = 0;
            sendPacket(&commandPacket);
            break;
    }
}*/

int main()
{
    // Connect to the Arduino
    startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);

    printf("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n");
    sleep(2);
    printf("DONE\n");

    // Spawn receiver thread
    pthread_t recv;
    pthread_create(&recv, NULL, receiveThread, NULL);

    // Send a hello packet
    TPacket helloPacket;
    helloPacket.packetType = PACKET_TYPE_HELLO;
    sendPacket(&helloPacket);

    // Put terminal into raw mode (no echo, no need for Enter)
    system("stty raw -echo");

    printf("Use W/A/S/D to move indefinitely, Space to stop, Q to quit.\n");

    exitFlag = 0;
    while(!exitFlag)
    {
        fd_set readfds;
        struct timeval tv;

        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        // Poll every 10 ms for better responsiveness
        tv.tv_sec = 0;
        tv.tv_usec = 10000; // 10,000 microseconds

        int rv = select(STDIN_FILENO+1, &readfds, NULL, NULL, &tv);

        if(rv > 0)
        {
            // There is a key available
            char c = getchar();
            TPacket cmd;
            cmd.packetType = PACKET_TYPE_COMMAND;

            switch(c)
            {
                case 'w':
                case 'W':
                    // Move forward indefinitely
                    cmd.command = COMMAND_FORWARD;
                    // params[0] = distance (0 => indefinite), params[1] = speed
                    cmd.params[0] = 0;
                    cmd.params[1] = 45;
		    //sendPacket(&cmd);
                    sendCommandWithCooldown(&cmd);
                    break;

                case 'a':
                case 'A':
                    // Turn left indefinitely
                    cmd.command = COMMAND_TURN_LEFT;
                    cmd.params[0] = 0;  // 0 => indefinite
                    cmd.params[1] = 90; // speed
		    //sendPacket(&cmd);
                    sendCommandWithCooldown(&cmd);
                    break;

                case 's':
                case 'S':
                    // Move backward indefinitely
                    cmd.command = COMMAND_REVERSE;
                    cmd.params[0] = 0;
                    cmd.params[1] = 45;
                    sendCommandWithCooldown(&cmd);
		    //sendPacket(&cmd);
                    break;

                case 'd':
                case 'D':
                    // Turn right indefinitely
                    cmd.command = COMMAND_TURN_RIGHT;
                    cmd.params[0] = 0;
                    cmd.params[1] = 90;
                    sendCommandWithCooldown(&cmd);
		    //sendPacket(&cmd);
                    break;

                case ' ':
                    // Space => STOP
                    cmd.command = COMMAND_STOP;
                    sendCommandWithCooldown(&cmd);
		    //sendPacket(&cmd);
                    break;

                case 'q':
                case 'Q':
                    exitFlag = 1;
                    break;
		case 'Z':
		case 'z':
		    cmd.command = COMMAND_OPEN_CLAW;
                    sendCommandWithCooldown(&cmd);
		    //sendPacket(&cmd);
		    break;

		case 'C':
		case 'c':
		    cmd.command = COMMAND_CLOSE_CLAW;
                    sendCommandWithCooldown(&cmd);
		    //sendPacket(&cmd);
		    break;
		case 'T':
		case 't': 
		    cmd.command = COMMAND_DEPLOY_MEDPACK;
                    sendCommandWithCooldown(&cmd);
		    //sendPacket(&cmd);
		    break;
		case 'R':
		case 'r':
		    cmd.command = COMMAND_RESET_MEDPACK;
                    sendCommandWithCooldown(&cmd);
		    //sendPacket(&cmd);
		    break;
                default:
                    // Unrecognized key => do nothing
                    break;
            }
        }
        // else: if no key was pressed in these 10 ms, we do nothing (no forced STOP).
    }

    // Restore normal terminal mode
    system("stty cooked echo");

    printf("\nClosing connection to Arduino.\n");
    endSerial();
    return 0;
}
	/*printf("Use WASD to move, Z for open claw, C for close. Press Q to quit. Release key to stop.\n");

	while (!exitFlag)
	{
		char ch = keyPressed();
		if (ch) {
			sendCommand(ch);
		}
		else {
		//if no new key, send STOP to ensure robot stops moving when we let go of the key
			sendCommand(' ');
		}
		usleep(100000);
	}	
	disableRawMode();
	printf("Closing connection to Arduino. \n");
	endSerial();
	return 0;*/

		
	
	//this is the original code, if needed revert back to this one
	/*while(!exitFlag)
	{
		char ch;
                    cmd.command = COMMAND_FORWARD;
                    // params[0] = distance (0 => indefinite), params[1] = speed
                    cmd.params[0] = 0;
                    cmd.params[1] = 40;
                    sendPacket(&cmd);
                    break;

                case 'a':
                case 'A':
                    // Turn left indefinitely
                    cmd.command = COMMAND_TURN_LEFT;
                    cmd.params[0] = 0;  // 0 => indefinite
                    cmd.params[1] = 60; */
