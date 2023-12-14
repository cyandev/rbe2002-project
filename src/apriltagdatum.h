#ifndef __APRILTAGDATUM_H
#define __APRILTAGDATUM_H

/*
 * AprilTagDatum is the data received from the camera 
*/
struct AprilTagDatum { 
    uint16_t id = 0; int32_t tx, ty, tz;
};

#endif
