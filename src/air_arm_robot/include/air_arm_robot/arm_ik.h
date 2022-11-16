#ifndef ARM_IK_H
#define ARM_IK_H

#include <iostream>
#include <cmath>

#define X1  -28.11
#define Y1   28.46
#define X2  -28.46
#define Y2  -28.11
#define Z    22.5
#define Zz   184.5

class Arm_IK{
public:
  Arm_IK() = default;
  double toRadian(double degree);
  double toDegree(double radian);

  double elbowRadian2Linear_Position(double pos_in_rad);
  double elbowRadian2Linear_Velocity(double pos_in_rad, double vel_in_rad);
  double elbowLinear2Radian_Position(double pos_in_mm);
  double elbowLinear2Radian_Velocity(double vel_in_mm);

  std::pair<double, double> wristEuler2LinearPosition(std::pair<double, double> desired_pos);
  std::pair<double, double> wristEuler2LinearVelocity(std::pair<double, double> desired_pos,
                                                      std::pair<double, double> desired_vel);
  std::pair<double, double> wristLinear2RadianPosition(std::pair<double, double> roll_pitch_in_mm);
  std::pair<double, double> wristLinear2RadianVelocity(std::pair<double, double> roll_pitch_vel_in_mm);
};

#endif // ARM_IK_H
