
#include <ros.h>
#include< std_msgs/Bool.h>

#define LED_RUNNING (11)
#define LED_WARNING (12)
#define LED_WAITING (13)


void _LEDRunningCb( const std_msgs::Bool& msg);
void _LEDWarningCb( const std_msgs::Bool& msg);
void _LEDWaitingCb( const std_msgs::Bool& msg);


ros::NodeHandle nh;
ros::Subscriber(<std_msgs::Bool> subLEDRunning("running_led", &_LEDRunningCb);
ros::Subscriber(<std_msgs::Bool> subLEDRunning("warning_led", &_LEDWarningCb);
ros::Subscriber(<std_msgs::Bool> subLEDRunning("waiting_led", &_LEDWaitingCb);


void setup() {
  pinMode(LED_RUNNING, OUTPUT);
  pinMode(LED_WARNING, OUTPUT);
  pinMode(LED_WAITING, OUTPUT);

  digitalWrite(LED_RUNNING,LOW);
  digitalWrite(LED_WARNING,LOW);
  digitalWrite(LED_WAITING,LOW);
  
  nh.initNode();
  nh.subscribe(sub);
}


void loop() {
  nh.spinOnce();
  delay(1);
}


void _LEDRunningCb( const std_msgs::Bool& msg) {
  digitalWrite(LED_RUNNING,msg.data);
}

void _LEDWarningCb( const std_msgs::Bool& msg) {
  digitalWrite(LED_WARNING,msg.data);
}

void _LEDWaitingCb( const std_msgs::Bool& msg) {
  digitalWrite(LED_WAITING,msg.data);
}
