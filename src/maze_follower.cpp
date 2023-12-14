#include <Romi32U4.h>
#include "Encoders.h"
#include "maze_follower.h"
#include "IR_sensor.h"
#include "IMU.h"

IRsensor ir;

void MazeFollower::init() {
  ir.Init();
}

double MazeFollower::getVelocity() {
  double distance = ir.ReadData();
  Serial.println(distance);
  double err = min(0,distance-MIN_DIST_CM);
  return FOLLOW_SPEED+err*50;
}

double MazeFollower::getAngularVelocity() {
  double distance = ir.ReadData();
  double err = distance-MIN_DIST_CM;
  if (err > 0 && abs(omega) > .1) {
    omega *= .9;
  } else {
    if (abs(omega) < OMEGA_MAX/2 && rand() % 3 == 0) {
      omega *= -1;
    }
    omega = omega < 0 ? -OMEGA_MAX : OMEGA_MAX;
  }

  return omega; //turn right
}