#pragma once

#include <Romi32U4.h>

class MazeFollower {
    private:
        const float Kp = 15; //Adapt parameters Kp and Kd until your robot consistently drives along a wall
        const float Kd = 2;
        const float MIN_DIST_CM = 25;
        const float OMEGA_MAX = 200; //mm/s
        const double FOLLOW_SPEED = 350; //mm/s
        float omega = 0;

    public:
        void init();
        
        //get velocity in mm/s
        double getVelocity(); 

        //get angular velocity in mm/s at outer wheel (too lazy for rad/s)
        double getAngularVelocity();
        
};