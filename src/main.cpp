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
enum ROBOT_STATE {RUNNING, SEEKING, CHASING};
ROBOT_STATE robot_state = RUNNING;

double X_FACTOR = 3.99600/100; //units -> cm, determined experimentally
double Z_FACTOR = 3.99600/100;

Romi32U4ButtonA buttonA;
SpeedController speedController;
MazeFollower mazeFollower;
OpenMV camera;
AprilTagDatum tag; //store the tag data for the last seen tag

void sendMessage(const String& topic, const String& message)
{
    Serial1.println(topic + String(':') + message);
}


void handleSerialEsp32(String s) {
  //todo: implement me :)
}

String esp32str;
void checkSerialEsp32()
{
    while(Serial1.available())
    {
        char c = Serial1.read();
        esp32str += c;

        if(c == '\n')
        {
            return handleSerialEsp32(esp32str);
            esp32str = "";
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
          double dist_cm = sqrt(pow(tag.tx*X_FACTOR, 2) + pow(tag.tz*Z_FACTOR, 2)); //find the distance in x/z plane; y doesn't matter
          double angle_rad = atan2(tag.tx*X_FACTOR,tag.tz*Z_FACTOR);

          Serial.print("found tag 4 dist = ");
          Serial.print(dist_cm);
          Serial.print(", angle: ");
          Serial.println(angle_rad);
        }
      }
    }

    return tagCount;
}

void setup() {
  //start serial
  Serial.begin(115200);
  Serial1.begin(115200);
  digitalWrite(0, HIGH); //internal pull up
  delay(100);

  //initialize controllers
  speedController.Init();
  mazeFollower.init();

  //initialize i2c
  Wire.begin();
  Wire.setClock(100000ul);

  //initialize robot state based on 
}

void loop() {
  //handle incoming mqtt messages

  //detect collisions

  //detect apriltags
  bool tagFound = findTag();

  //update forward kinematics

  //handle robot state
  switch (robot_state) {
    case RUNNING: {
      //todo: implement me
      double v = mazeFollower.getVelocity();
      double omega = mazeFollower.getAngularVelocity();
      speedController.Process(v-omega, v+omega);
      break;
    }
    
    case SEEKING: {
      double v = mazeFollower.getVelocity();
      double omega = mazeFollower.getAngularVelocity();
      speedController.Process(v-omega, v+omega);
      if (tagFound) {
        robot_state = CHASING;
        //todo: reset FK here
      }
      break;
    }

    case CHASING: {
      if (!tagFound) {
        robot_state = SEEKING; //TODO: replace with FK-based chasing
        break;
      }

      double dist_cm = sqrt(pow(tag.tx*X_FACTOR, 2) + pow(tag.tz*Z_FACTOR, 2)); //find the distance in x/z plane; y doesn't matter
      double angle_rad = atan2(tag.tx*X_FACTOR,tag.tz*Z_FACTOR);
      double u_angular = 500 * angle_rad;
      double u_linear = 50 * (dist_cm) * cos(angle_rad); //fancy easy way to make it move kinda nice

      speedController.Process(u_linear-u_angular, u_linear+u_angular);   
    }
  }
  delay(10);
}