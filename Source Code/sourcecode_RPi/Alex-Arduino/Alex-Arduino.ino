/*
 * CG2111A: ALEX TO THE RESCUE
 * ALEX ARDUINO UNO SOURCE CODE
 * DONE BY: B04-1B
 * 
 * Additional Functionalities:
 * Ultrasonic Sensor (Reverse distance)
 * Control Alex over TLS secured connection on laptop 
 * (tls-alex-server.cpp and tls-alex-client.cpp)
 * Bare-metal
 */

#include <serialize.h>
#include <math.h>
#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <avr/io.h>
#include <avr/interrupt.h>

typedef enum
{
  STOP = 0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.
#define COUNTS_PER_REV      4

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC
#define WHEEL_CIRC          1

// PI for calculating turn circumference
#define PI 3.141592654

// Alex's length and breadth in cm
#define ALEX_LENGTH 18
#define ALEX_BREADTH 13

// Alex's diagonal length, computed in setup()
float AlexDiagonal = 0.0;

// Alex's turning circumference, computed in setup()
float AlexCirc = 0.0;

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  1 << 1   // Left forward pin (PORT B)
#define LR                  1 << 2   // Left reverse pin (PORT B)
#define RF                  1 << 6  // Right forward pin (PORT D)
#define RR                  1 << 5  // Right reverse pin (PORT D)


/*
 *    Alex's State Variables
 */
// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and Right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance travelled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

// Variables used for measuring Ultrasonic Distance
#define TRIGPIN     1 << 4 // pin 12 PB4
#define ECHOPIN     1 << 5 // pin 13 PB5
float SPEED_OF_SOUND = 0.0345; // Defines the speed of sound for measurement


/*
 * Additional Feature: Ultrasonic Distance
 * Measures the distance away an obstacle is from the back of the robot
 * Added to sendStatus() function to obtain reading
 */
// set up the pins for the Ultrasonic Sensor
void setupUltra()
{
  DDRB |= (TRIGPIN); // set TRIGPIN as output to emit pulse
  DDRB &= ~(ECHOPIN); // set ECHOPIN as input to receive pulse
}

// Function to measure distance from Ultrasonic
unsigned long measureUltra()
{
  // send pulse
  PORTB |= (TRIGPIN);
  delay(10);
  PORTB &= ~(TRIGPIN);

  // receive pulse
  float microsecs = pulseIn(13, HIGH);
  float cms = microsecs * SPEED_OF_SOUND / 2;
  return (unsigned long) cms;
}


/*
 * Alex Communication Routines. 
 */ 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.

  TPacket statusPacket;
  statusPacket.packetType=PACKET_TYPE_RESPONSE;
  statusPacket.command=RESP_STATUS;
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
  statusPacket.params[10] = measureUltra();
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...) 
{
  // print function used for debugging
  
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  
  DDRD &= ~(0b00001100); //set pin 2 and 3 to input
  PORTD |= 0b00001100; //sets pin 2 and 3 HIGH
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  switch(dir) 
  {
    case FORWARD:
      leftForwardTicks++;
      break;
    
    case BACKWARD:
      leftReverseTicks++;
      break;
 
    case LEFT:
      leftReverseTicksTurns++;
      break;
    
    case RIGHT:
      leftForwardTicksTurns++;
      break;
  }

  if (dir == FORWARD)
    forwardDist = (unsigned long)((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);

  if (dir == BACKWARD)
    reverseDist = (unsigned long)((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
}

void rightISR()
{
  switch(dir) 
  {
    case FORWARD:
      rightForwardTicks++;
      break;
    
    case BACKWARD:
      rightReverseTicks++;
      break;
 
    case LEFT:
      rightForwardTicksTurns++;
      break;
    
    case RIGHT:
      rightReverseTicksTurns++;
      break;
  }
}

void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered
  // Enabled INT0 and INT1 interrupts.
  
  EICRA = 0b00001010;
  EIMSK = 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}


/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection.
void setupSerial()
{
  Serial.begin(9600);
}

// Start the serial connection.
void startSerial()
{
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 

int readSerial(char *buffer)
{
  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port.
void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}


/*
 * Alex's motor drivers.
 */
// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B (RR)
   *    A2IN - Pin 6, PD6, OC0A (RF)
   *    B1IN - Pin 9, PB1, OC1A (LF)
   *    B2IN - pIN 10, PB2, OC1B (LR)
   */
   DDRB |= (LF | LR); // Pin 9 and 10 as output
   DDRD |= (RF | RR); // Pin 5 and 6 as output
}

// Start the PWM for Alex's motors.
void startMotors()
{
   // PWM setup for A1IN and A2IN
   TCNT0 = 0;
   TCCR0A = 0b00000001; // Initially disables 0CR0A and 0CR0B
   TCCR0B = 0b00000001; // PWM, Phase correct, 1 prescaler

   // PWM setup for B1IN and B2IN
   TCNT1 = 0;
   TCCR1A = 0b00000001; // Initially disables 0CR1A  and 0CR1B
   TCCR1B = 0b00000001; // PWM, Phase Correct, 8-bit, 1 prescaler
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward relative "dist" at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// "dist" 20 is measured to be ~13cm (half a tile)
void forward(float dist, float speed)
{  
  dir = FORWARD; 

  // code to tell us how far to move
  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist=9999999;

  newDist = forwardDist + deltaDist;
  
  int val = pwmVal(speed); // controls speed of Alex

  // motor control
  OCR0A = val;
  TCCR0A = 0b10000001; // activate RF
  OCR1A = val;
  TCCR1A = 0b10000001; // activate LF
}

// Reverse Alex relative "dist" at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// "dist" 20 is measured to be ~13cm
void reverse(float dist, float speed)
{
  dir = BACKWARD;

  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;

  newDist = reverseDist + deltaDist;

  int val = pwmVal(speed); // controls speed of Alex

  // motor control
  OCR0B = val;
  TCCR0A = 0b00100001; // activate RR
  OCR1B = val;
  TCCR1A = 0b00100001; // activate LR
}

// Function to estimate number of wheel ticks needed to turn an angle
unsigned long computeDeltaTicks(float ang)
{
  // We will assume that angular distance moved = linear distance moved in one wheel
  //revolution. This is (probably) incorrect but simplifies calculation.
  // # of wheel revs to make one full 360 turn is alexCirc / WHEEL_CIRC
  // This is for 360 degrees. For ang degrees it will be (ang*alexCirc) / (360 * WHEEL_CIRC)
  // To convert to ticks, we multiply by COUNTS_PER_REV

  unsigned long ticks = (unsigned long)((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.

void left(float ang, float speed)
{
  int val = pwmVal(speed);
  
  dir = LEFT;

  if (ang == 0)
    deltaTicks = 9999999;
  else
  {
    deltaTicks = computeDeltaTicks(ang);
  }

  targetTicks = rightReverseTicksTurns + deltaTicks;

  // motor control
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  OCR0A = val;
  TCCR0A = 0b10000001; // activate RF
  OCR1B = val;
  TCCR1A = 0b00100001; // activate LR
}


void right(float ang, float speed)
{
  int val = pwmVal(speed);
  
  dir = RIGHT;
  
  if (ang == 0)
    deltaTicks = 9999999;
  else
  {
    deltaTicks = computeDeltaTicks(ang);
  }

  targetTicks = leftReverseTicksTurns + deltaTicks;

  // motor control
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  OCR0B = val;
  TCCR0A = 0b00100001; // activate RR
  OCR1A = val;
  TCCR1A = 0b10000001; // activate LF
}


// Stop Alex
void stop()
{
  dir = STOP;

  // motor control
  TCCR0A = 0b00000001; // disables OCR0A and OCR0B
  TCCR1A = 0b00000001; // disables OCR1A and OCR1B
}


/*
 * Alex's setup and run codes
 * 
 */
// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

// Attached to hotkey 'W'
// Auto moves Alex forward by half a tile (~13cm)
void forwardmacro() {
  forward(20,75);
}

// Attached to hotkey 'S'
// Auto moves Alex backwards by half a tile (~13cm)
void backwardmacro() {
  reverse(20,75);
}

// Attached to hotkey 'A'
// Auto rotates Alex to the left by 20 degrees
void leftmacro() {
  left(20,75);
}

// Attached to hotkey 'D'
// Auto rotates Alex to the right by 20 degrees
void rightmacro() {
  right(20,75);
}

// Attached to hotkey 'T'
// Uses higher speed and distance to get Alex over the hump
// Auto moves Alex at a higher speed by 1.5 tiles (~39cm)
void turbomacro(){
  forward(60, 90);
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;

    /*
     * Implement code for other commands here.
     * 
     */
    case COMMAND_GET_STATS:
       sendStatus();
       break;

    case COMMAND_CLEAR_STATS:
       sendOK();
       clearOneCounter(command->params[0]);
       break;
     
    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    // Additional commands
    case COMMAND_W:
      sendOK();
      forwardmacro();
      break;

    case COMMAND_S:
      sendOK();
      backwardmacro();
      break;

    case COMMAND_A:
      sendOK();
      leftmacro();
      break;
      
    case COMMAND_D:
      sendOK();
      rightmacro();
      break;
    case  COMMAND_T:
      sendOK();
      turbomacro();
      break;
    
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  // compute the diagonal
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;
  
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  setupUltra();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
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

void loop() {
 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 

  // stops Alex once it has reached the distance required
  if (deltaDist > 0)
  {
    if (dir == FORWARD)
    {
      if (forwardDist >= newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD)
    {
        if (reverseDist >= newDist)
        {
          deltaDist = 0;
          newDist = 0;
          stop();
        }
    }
    else if (dir == STOP)
    {
      deltaDist = 0;
      newDist = 0;
      stop();
    } 
  }

  if (deltaTicks > 0)
  {
    if (dir == LEFT)
    {
      if (leftReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT)
    {
      if (rightReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == STOP)
    {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}
