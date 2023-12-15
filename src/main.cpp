#include <Arduino.h>
#include <Romi32U4.h>
//sensors
#include "IR_sensor.h"
#include "Encoders.h"
#include "openmv.h"
#include <Wire.h>
//behaviors
#include "Speed_controller.h"
#include "maze_follower.h"
#include "IMU.h"
enum ROBOT_STATE {RUNNING, SEEKING, CHASING};
ROBOT_STATE robot_state = RUNNING;

double X_FACTOR = 3.99600/100; //units -> cm, determined experimentally
double Z_FACTOR = 3.99600/100;

Romi32U4ButtonA buttonA;
SpeedController speedController;
MazeFollower mazeFollower;
OpenMV camera;
IMU_sensor lsm6;
AprilTagDatum tag; //store the tag data for the last seen tag

unsigned long last_collision = millis();

unsigned long last_tag = millis();
void sendMessage(const String& topic, const String& message)
{
    Serial1.println(topic + String(':') + message);
}


String esp32str = "";
void checkSerialEsp32()
{
    while(Serial1.available())
    {
        char c = Serial1.read();

        if (c == '\r') continue; //no carriage returns

        if(c == '\n')
        {
          Serial.println(esp32str);
          String topic = esp32str.substring(0,esp32str.indexOf(':'));
          String value = esp32str.substring(esp32str.indexOf(':') + 1);

          Serial.println(topic);
          Serial.println(value);
          if (topic.equals("it")) {
              if (value == String(_ROBOT_NUM)) {
                Serial.println("state changed to seeking");
                robot_state = SEEKING;
              } else {
                Serial.println("state changed to running");
                robot_state = RUNNING;
              }
            }

            if (topic == "collision" && (robot_state == SEEKING || robot_state == CHASING)) {
              Serial.println("collision");
              Serial.println(millis()-last_collision);
              if (millis()-last_collision < 500) {
                Serial.println("tag ur it");
                sendMessage("it", value);
                robot_state = RUNNING;
              }
            }

          esp32str = "";
        } else {
          esp32str += c;
        }
    }
}

bool findTag()
{
    uint8_t tagCount = camera.getTagCount();
    if(tagCount) 
    {
      if(camera.readTag(tag))
      {
        if (tag.id == 4) {
          last_tag = millis();
          return true;
        }
      }
    }

    return false;
}

void setup() {
  //start serial
  Serial.begin(115200);
  Serial1.begin(115200);
  digitalWrite(0, HIGH); //internal pull up

  //initialize i2c
  Wire.begin();
  Wire.setClock(100000ul);

  //initialize controllers
  speedController.Init();
  mazeFollower.init();
  lsm6.Init();
  
  robot_state = RUNNING;
}

void loop() {
  //handle incoming mqtt messages
  checkSerialEsp32();

  //detect collisions
  auto acc = lsm6.ReadAcceleration();
  if ((acc.X*0.061 > 500 || acc.Y*0.061 > 500) && millis()-last_collision > 1000) {
    Serial.println("ow");
    last_collision = millis();
    if (robot_state == RUNNING) {
      sendMessage("collision", String(_ROBOT_NUM));
    }
  }

  //detect apriltags
  bool tagFound = findTag();

  //update forward kinematics

  //handle robot state
  switch (robot_state) {
    case RUNNING: {
      mazeFollower.setVelocity(250);
      //todo: implement me
      double v = mazeFollower.getVelocity();
      double omega = mazeFollower.getAngularVelocity();
      speedController.Process(v-omega, v+omega);
      break;
    }
    
    case SEEKING: {
      mazeFollower.setVelocity(100);
      double v = mazeFollower.getVelocity();
      double omega = mazeFollower.getAngularVelocity();
      speedController.Process(v-omega, v+omega);
      if (tagFound) {
        sendMessage("tag", "x: " + String(tag.tx) + ", z: " + String(tag.tz));
        robot_state = CHASING;
        Serial.println("Chasing");
        //todo: reset FK here
      }
      break;
    }

    case CHASING: {
      if (!tagFound && millis() - last_tag > 1000) { //for now, go back to seeking after 250ms
        robot_state = SEEKING; //TODO: replace with FK-based chasing
        Serial.println("Seeking");
        break;
      }
      
      double angle_rad = atan2(tag.tx*X_FACTOR,tag.tz*Z_FACTOR);
      double u_angular = 500 * angle_rad;
      double u_linear = 350 * cos(angle_rad); //fancy easy way to make it move kinda nice, send it at the target 

      speedController.Process(u_linear-u_angular, u_linear+u_angular);   
    }
  }
  delay(5);
}