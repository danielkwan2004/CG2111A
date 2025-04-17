
//#include <Multiplexer.h>
//#include <ArduinoRobotMotorBoard.h>
//#include <LineFollow.h>
//#include <EasyTransfer2.h>
#define S0 27
#define S1 26
#define S2 25
#define S3 24
#define sensorOut 28
#include <math.h>
#include <stdarg.h>
#include <serialize.h>

#include "packet.h"
#include "constants.h"
#include <Servo.h>

//for color sensor
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;
//end of color sensor

volatile TDirection dir;
/*
   Alex's configuration constants
*/

// PI, for calculating circumference
//#define PI 3.141592654

// Alex's length and breadth in cm
#define ALEX_LENGTH 25.6
#define ALEX_BREADTH 14.9

// Alex's Diagonal
float alexDiagonal = 0.0;
//Alex's turning circumference
float alexCirc = 0.0;


// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV 4.0

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC 19.782

/*
      Alex's State Variables
*/

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;
// Variables to keep track of whether we've moved
// a command distance
unsigned long deltaDist;
unsigned long newDist;

//Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

//take from the front 
#define clawLeftAbovepin 9
#define clawRightAbovepin 10
#define clawLeftBelowpin 44  
#define clawRightBelowpin 45
#define CLAWMEDPACKPIN 46
Servo clawLeftAbove;
Servo clawRightAbove;
Servo clawLeftBelow;
Servo clawRightBelow;
Servo medpackclaw;

/*

   Alex Communication Routines.

*/

// New function to estimate number of wheel ticks
// needed to turn an angle
unsigned long computeDeltaTicks(float ang) {
  // We will assume that angular distance moved = linear distance moved in one wheel
  // revolution. THis is (probably) incorrect but simplifies calculation.
  // # of wheel revs to make one full 360 turn is alexCirc / WHEEL_CIRC
  // This is for 360 degrees. For ang degrees it will be (ang * alexCirc) / (360 * WHEEL_CIRC)
  // To convert to ticks, we multiply by COUNTS_PER_REV

  unsigned long ticks = (unsigned long)((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

void left(float ang, float speed) {

  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = leftReverseTicksTurns + deltaTicks;
  ccw(ang, speed);
}

void right(float ang, float speed) {
  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = rightReverseTicksTurns + deltaTicks;
  cw(ang, speed);
}

TResult readPacket(TPacket *packet) {
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);
}

void sendStatus() {
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  sendResponse(&statusPacket);

  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
}

void sendMessage(const char *message) {
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendColorMessage(const char *color) {
  TPacket colorPacket;
  colorPacket.packetType = PACKET_TYPE_COLOR;
  strncpy(colorPacket.data, color, MAX_STR_LEN);
  sendResponse(&colorPacket);
}

void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket() {
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum() {
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand() {
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse() {
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK() {
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet) {
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
   Setup and start codes for external interrupts and
   pullup resistors.

*/
// Enable pull up resistors on pins 18 and 19
void enablePullups() {
  DDRD &= ~((1 << 2) | (1 << 3));
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  PORTD |= (1 << 2) | (1 << 3);
  //Driving the Pin high by writing a 1(HIGH) to it
}

// Functions to be called by INT2 and INT3 ISRs.
void leftISR() {
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long)((float)leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);

  } else if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long)((float)leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  } else if (dir == LEFT) {
    leftReverseTicksTurns++;
  }
  //  Serial.print("LEFT: ");
  //  Serial.println(leftTicks / COUNTS_PER_REV * WHEEL_CIRC);
}

void rightISR() {
  if (dir == FORWARD) {
    rightForwardTicks++;
  } else if (dir == BACKWARD) {
    rightReverseTicks++;
  } else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  } else if (dir == LEFT) {
    rightForwardTicksTurns++;
  }
  //  Serial.print("RIGHT: ");
  //  Serial.println(rightTicks / COUNTS_PER_REV * WHEEL_CIRC);
}

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT() {

  EICRA |= 0b10100000;
  EIMSK |= 0b00001100;
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.
}

ISR(INT2_vect) {
  rightISR();
}

ISR(INT3_vect) {
  leftISR();
}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.


// Implement INT2 and INT3 ISRs above.

/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial() {
  // To replace later with bare-metal.
  Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial() {
  // Empty for now. To be replaced with bare-metal code
  // later on.
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer) {

  int count = 0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len) {
  Serial.write(buffer, len);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
   Alex's setup and run codes

*/

// Clears all our counters
void clearCounters() {
  leftForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  rightForwardTicks = 0;
  leftForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightForwardTicksTurns = 0;
  rightReverseTicksTurns = 0;
  leftRevs = 0;
  rightRevs = 0;
  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which) {
  //  switch(which)
  //  {
  //    case 0:
  //      clearCounters();
  //      break;
  //
  //    case 1:
  //      leftTicks=0;
  //      break;
  //
  //    case 2:
  //      rightTicks=0;
  //      break;
  //
  //    case 3:
  //      leftRevs=0;
  //      break;
  //
  //    case 4:
  //      rightRevs=0;
  //      break;
  //
  //    case 5:
  //      forwardDist=0;
  //      break;
  //
  //    case 6:
  //      reverseDist=0;
  //      break;
  clearCounters();
}

// Intialize Alex's internal states

void initializeState() {
  clearCounters();
}

void handleCommand(TPacket *command) {
  switch (command->command) {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((double)command->params[0], (float)command->params[1]);
      break;

    case COMMAND_REVERSE:
      backward((double)command->params[0], (float)command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
      left((double)command->params[0], (float)command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
      right((double)command->params[0], (float)command->params[1]);
      break;
    case COMMAND_OPEN_CLAW:
      clawOpen();
      break;
    case COMMAND_CLOSE_CLAW:
      clawClose();
      break;
    case COMMAND_DEPLOY_MEDPACK:
      deploymedpack();
      break;
    case COMMAND_RESET_MEDPACK:
      resetmedpack();
      break;
    case COMMAND_STOP:
      stop();
      break;
    case COMMAND_GET_STATS:
      sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
      clearOneCounter(command->params[0]);
      sendOK();
      break;

    default:
      sendBadCommand();
  }
}

void waitForHello() {
  int exit = 0;

  while (!exit) {
    TPacket hello;
    TResult result;

    do {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK) {
      if (hello.packetType == PACKET_TYPE_HELLO) {


        sendOK();
        exit = 1;
      } else
        sendBadResponse();
    } else if (result == PACKET_BAD) {
      sendBadPacket();
    } else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  }  // !exit
}

void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  initializeState();
  sei();

  clawLeftAbove.attach(9);  // adjust pin numbers as needed
  clawRightAbove.attach(10);
  clawLeftBelow.attach(clawLeftBelowpin); //fill pin numbers
  clawRightBelow.attach(clawRightBelowpin); //fill pin numbers
  medpackclaw.attach(CLAWMEDPACKPIN);


  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);  // 20% frequency scaling
}



void handlePacket(TPacket *packet) {
  switch (packet->packetType) {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
        break;
    }
}

//color sensor constants
const float ratioThreshold       = 0.05;  // One ratio must exceed the other by at least this
const unsigned long brightnessThreshold = 10; // If total < this, "no color"
const unsigned long colorCheckInterval  = 100; // ms between checks

// State variables to track color checks and last color
unsigned long lastColorCheck       = 0;
char lastColor[MAX_STR_LEN]        = ""; // previously detected color

static unsigned long lastColorSendTime = 0;
const unsigned long colorSendInterval = 200;
//end of color sensor constants
#define COLOR_BUF_SIZE 5
char colorBuffer[COLOR_BUF_SIZE][MAX_STR_LEN];
int colorIndex = 0;  // Points to the next write position

// This holds the last color we actually sent to the Pi.
char lastSentColor[MAX_STR_LEN] = "no color detected right now";

// 2) Function to figure out which color appears most often in the ring buffer
//    (We only do "RED", "GREEN", or "no color detected right now" in this example.)
const char* computeMajorityColor()
{
  // We'll just count how many times we see "RED", "GREEN", or anything else
  int countRed = 0;
  int countGreen = 0;
  
  for(int i = 0; i < COLOR_BUF_SIZE; i++) {
    if(strcmp(colorBuffer[i], "RED") == 0) {
      countRed++;
    }
    else if(strcmp(colorBuffer[i], "GREEN") == 0) {
      countGreen++;
    }
  }

  // Whichever is highest wins, or "no color detected right now" if there's a tie
  if(countRed > countGreen && countRed > (COLOR_BUF_SIZE/2)) {
    return "RED";
  }
  else if(countGreen > countRed && countGreen > (COLOR_BUF_SIZE/2)) {
    return "GREEN";
  }
  else {
    return "no color detected right now";
  }
}

void loop() {
  // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2
  //  clawOpen();
  //  delay(2000);
  //   clawClose();
  //  delay(2000);
  //forward(0, 100);
  //  dbprintf("PI is %3.2f\n", PI);
  // put your main code here, to run repeatedly:
  // right(0,100);
  // delay(1000);
  // stop();
  // backward(0,100);
  // stop();
  // delay(5000);
  // ccw(0,100);
  // stop();
  // delay(5000);
  // cw(0,100);
  // stop();
  // delay(5000);
    // 1) If there's text in the Serial buffer, read it as a line
  // deploymedpack();
  // delay(2000);
  // resetmedpack();
  // delay(2000);
  TPacket recvPacket;  // This holds commands from the PI
  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD) {
    sendBadPacket();
  } else if (result == PACKET_CHECKSUM_BAD) {
    sendBadChecksum();
  }

  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == BACKWARD) {
      if (reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if ((Tdir)dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }
  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }

    } else if ((Tdir)dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
  unsigned long now = millis();
  if (now - lastColorCheck >= colorCheckInterval) {
    lastColorCheck = now; // reset the timer

    // 2) Read RED
    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    redFrequency = pulseIn(sensorOut, LOW);

    // 3) Read GREEN
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    greenFrequency = pulseIn(sensorOut, LOW);

    // 4) Read BLUE (optional)
    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    blueFrequency = pulseIn(sensorOut, LOW);

    // 5) Compute sum & ratios
    unsigned long sum = redFrequency + greenFrequency + blueFrequency;
    if (sum < 1) sum = 1; // protect against /0
    float rRatio = (float) redFrequency   / sum;
    float gRatio = (float) greenFrequency / sum;
    float bRatio = (float) blueFrequency / sum;
    // (We won't specifically use bRatio for "red vs. green," 
    // but reading it helps with total brightness.)

    // 6) Determine newColor
    char newColor[MAX_STR_LEN];
    strncpy(newColor, "no color detected right now", MAX_STR_LEN);

    // If total brightness is high enough, compare red vs. green
    if (sum >= brightnessThreshold) {
      // Must exceed the other ratio by ratioThreshold
      if (rRatio > gRatio + 0.06 && rRatio > bRatio + 0.06) {
        strncpy(newColor, "RED", MAX_STR_LEN);
      } 
      else if (gRatio > rRatio + 0.06 && gRatio > bRatio + 0.06) {
        strncpy(newColor, "GREEN", MAX_STR_LEN);
      }
      // else remains "no color detected right now"
    }

    strncpy(colorBuffer[colorIndex], newColor, MAX_STR_LEN);
    colorIndex = (colorIndex + 1) % COLOR_BUF_SIZE;

    // 5) Compute the stable color from the buffer
    const char* stableColor = computeMajorityColor();

    // 6) Only send if stable color changed from last time
    if(strcmp(stableColor, lastSentColor) != 0) {
      sendColorMessage(stableColor);
      strncpy(lastSentColor, stableColor, MAX_STR_LEN);
    }
  }



}
