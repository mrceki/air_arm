#ifndef MOVE_GROUP_INTERFACE_TUTORIAL_H
#define MOVE_GROUP_INTERFACE_TUTORIAL_H
#include <math.h>

namespace brkygkcn {

double toRadian(double degrees){
  return degrees * M_PI / 180.0;
}

double toDegree(double radian){
  return radian * 180.0 / M_PI;
}

}
#endif // MOVE_GROUP_INTERFACE_TUTORIAL_H
