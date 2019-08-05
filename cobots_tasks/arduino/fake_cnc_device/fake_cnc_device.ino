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

#include <ros.h>
#include<std_msgs/Bool.h>

//==============================================================================
//  Constants and Macro Declarations
//==============================================================================

#define LED_RUNNING (11)
#define LED_WARNING (12)
#define LED_WAITING (13)

//==============================================================================
//  Private Function Prototypes
//==============================================================================

static void _LEDRunningCb( const std_msgs::Bool& msg);
static void _LEDWarningCb( const std_msgs::Bool& msg);
static void _LEDWaitingCb( const std_msgs::Bool& msg);

//==============================================================================
//  Private Data Members
//==============================================================================

static ros::NodeHandle nh;
static ros::Subscriber<std_msgs::Bool> _subLEDRunning("running_led", &_LEDRunningCb);
static ros::Subscriber<std_msgs::Bool> _subLEDWarning("warning_led", &_LEDWarningCb);
static ros::Subscriber<std_msgs::Bool> _subLEDWaiting("waiting_led", &_LEDWaitingCb);

//==============================================================================
//  Main
//==============================================================================

void setup() {
  pinMode(LED_RUNNING, OUTPUT);
  pinMode(LED_WARNING, OUTPUT);
  pinMode(LED_WAITING, OUTPUT);

  digitalWrite(LED_RUNNING,LOW);
  digitalWrite(LED_WARNING,LOW);
  digitalWrite(LED_WAITING,LOW);

  nh.initNode();
  nh.subscribe(_subLEDRunning);
  nh.subscribe(_subLEDWarning);
  nh.subscribe(_subLEDWaiting);
}

void loop() {
  nh.spinOnce();
  delay(1);
}

//==============================================================================
//  Private Function Implementation
//==============================================================================

static void _LEDRunningCb( const std_msgs::Bool& msg) {
  digitalWrite(LED_RUNNING,(msg.data) ? (HIGH) : (LOW));
}

static void _LEDWarningCb( const std_msgs::Bool& msg) {
  digitalWrite(LED_WARNING,(msg.data) ? (HIGH) : (LOW));
}

static void _LEDWaitingCb( const std_msgs::Bool& msg) {
  digitalWrite(LED_WAITING,(msg.data) ? (HIGH) : (LOW));
}
