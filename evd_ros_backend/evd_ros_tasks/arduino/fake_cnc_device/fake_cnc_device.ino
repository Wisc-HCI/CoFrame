/**
 * Fake CNC Device
 * @author Curt Henrichs
 * @date 7-25-19
 *
 * Defines the physical fake interface for the fake CNC machine.
 *
 * Used to create a more immersive interaction as it provides visual indication
 * of the CNCs status for the operator.
 */

//==============================================================================
//  Libraries
//==============================================================================

#include <Servo.h> 
#include <ros.h>
#include<std_msgs/Bool.h>
#include <std_msgs/UInt16.h>

//==============================================================================
//  Constants and Macro Declarations
//==============================================================================

#define LED_RUNNING_PIN             (11)
#define LED_WARNING_PIN             (12)
#define LED_WAITING_PIN             (13)
#define DOOR_SERVO_PIN              (10)

#define DOOR_SERVO_POSTION_OPEN     (45)
#define DOOR_SERVO_POSTION_CLOSE    (170)
#define DOOR_SERVO_POSTION_DEFAULT  DOOR_SERVO_POSTION_CLOSE

//==============================================================================
//  Private Function Prototypes
//==============================================================================

static void _LEDRunningCb( const std_msgs::Bool& msg);
static void _LEDWarningCb( const std_msgs::Bool& msg);
static void _LEDWaitingCb( const std_msgs::Bool& msg);
static void _doorServoCb( const std_msgs::UInt16& msg);

//==============================================================================
//  Private Data Members
//==============================================================================

static Servo _doorServo;

static ros::NodeHandle nh;
static ros::Subscriber<std_msgs::Bool> _subLEDRunning("running_led", &_LEDRunningCb);
static ros::Subscriber<std_msgs::Bool> _subLEDWarning("warning_led", &_LEDWarningCb);
static ros::Subscriber<std_msgs::Bool> _subLEDWaiting("waiting_led", &_LEDWaitingCb);
static ros::Subscriber<std_msgs::UInt16> _subDoorServo("door_servo", &_doorServoCb);

//==============================================================================
//  Main
//==============================================================================

void setup() {
  pinMode(LED_RUNNING_PIN, OUTPUT);
  pinMode(LED_WARNING_PIN, OUTPUT);
  pinMode(LED_WAITING_PIN, OUTPUT);

  digitalWrite(LED_RUNNING_PIN,LOW);
  digitalWrite(LED_WARNING_PIN,LOW);
  digitalWrite(LED_WAITING_PIN,LOW);

  _doorServo.attach(DOOR_SERVO_PIN);
  _doorServo.write(DOOR_SERVO_POSTION_DEFAULT);

  nh.initNode();
  nh.subscribe(_subLEDRunning);
  nh.subscribe(_subLEDWarning);
  nh.subscribe(_subLEDWaiting);
  nh.subscribe(_subDoorServo);
}

void loop() {
  nh.spinOnce();
  delay(1);
}

//==============================================================================
//  Private Function Implementation
//==============================================================================

static void _LEDRunningCb( const std_msgs::Bool& msg) {
  digitalWrite(LED_RUNNING_PIN,(msg.data) ? (HIGH) : (LOW));
}

static void _LEDWarningCb( const std_msgs::Bool& msg) {
  digitalWrite(LED_WARNING_PIN,(msg.data) ? (HIGH) : (LOW));
}

static void _LEDWaitingCb( const std_msgs::Bool& msg) {
  digitalWrite(LED_WAITING_PIN,(msg.data) ? (HIGH) : (LOW));
}

static void _doorServoCb( const std_msgs::UInt16& msg) {
  _doorServo.write(msg.data);
}

