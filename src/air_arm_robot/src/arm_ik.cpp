#include "air_arm_robot/arm_ik.h"

double Arm_IK::toRadian(double degree){

  return (degree / 180.0) * M_PI;
}

double Arm_IK::toDegree(double radian){

  return (radian / M_PI) * 180.0;
}

std::pair<double, double> Arm_IK::wristEuler2LinearPosition(std::pair<double, double> desired_pos){

  double r = desired_pos.first; // in radian
  double p = desired_pos.second; // in radian

  double C1 = cos(r);
  double C2 = cos(p);
  double S1 = sin(r);
  double S2 = sin(p);

  //in mm
  double pitch_command = 162.0 - sqrt(pow(X1-X1*C2,2) + pow((Y1-(X1*S1*S2+Y1*C1)),2) +
                              pow((Z-(Zz-X1*C1*S2+Y1*S1)),2));
  //in mm
  double roll_command = 162.0 - sqrt(pow(X2-X2*C2,2) + pow((Y2-(X2*S1*S2+Y2*C1)),2) +
                              pow((Z-(Zz-X2*C1*S2+Y2*S1)),2));

  return std::make_pair (roll_command,pitch_command);
}

std::pair<double, double> Arm_IK::wristEuler2LinearVelocity(std::pair<double, double> desired_pos,
                                                            std::pair<double, double> desired_vel){

  double th_roll  = desired_pos.first;
  double th_pitch = desired_pos.second;
  double th_dot_roll  = desired_vel.first;
  double th_dot_pitch = desired_vel.second;

  double C1 = cos(th_roll);
  double C2 = cos(th_pitch);
  double S1 = sin(th_roll);
  double S2 = sin(th_pitch);

  double pitchL = 162.0 - sqrt(pow(X1-X1*C2,2) + pow((Y1-(X1*S1*S2+Y1*C1)),2) +
                              pow((Z-(Zz-X1*C1*S2+Y1*S1)),2));
  double rollL = 162.0 - sqrt(pow(X2-X2*C2,2) + pow((Y2-(X2*S1*S2+Y2*C1)),2) +
                              pow((Z-(Zz-X2*C1*S2+Y2*S1)),2));

  double pitch_vel_cmd = (2*X1*X1*S2*th_dot_pitch - 2*Y1*X1*C1*S2*th_dot_roll - 2*Y1*X1*S1*C2*th_dot_pitch +
                    2*Y1*Y1*S1*th_dot_roll -2*Z*X1*S1*S2*th_dot_roll + 2*Z*X1*C1*C2*th_dot_pitch - 2*Z*Y1*C1*th_dot_roll +
                    2*Zz*X1*S1*S2*th_dot_roll - 2*Zz*X1*C1*C2*th_dot_pitch + 2*Zz*Y1*C1*th_dot_roll)/(2*(pitchL - 162.0));


  double roll_vel_cmd = (2*(X2 - X2*C2)*X2*S2*th_dot_pitch + 2*(Y2 - (X2*S1*S2 + Y2*C1))*(Y2*S1*th_dot_roll -
                  X2*C1*S2*th_dot_roll - X2*S1*C2*th_dot_pitch) + 2*((Z - Zz) +
                  (X2*C1*S2 - Y2*S1))*(-X2*S1*S2*th_dot_roll + X2*C1*C2*th_dot_pitch - Y2*C1*th_dot_roll))/(2*(rollL - 162.0));


  return std::make_pair(roll_vel_cmd, pitch_vel_cmd);

}

std::pair<double, double> Arm_IK::wristLinear2RadianPosition(std::pair<double, double> roll_pitch_in_mm){

   double rollxpiston = roll_pitch_in_mm.first;
   double pitchypiston = roll_pitch_in_mm.second;

   // pitch coefficients

   double p00 = 0.001665;
   double p10 = 1;
   double p01 = 1.023;
   double p20 = 0.001848;
   double p11 = -0.001393;
   double p02 = 0.0004754;
   double p30 = 3.407e-05;
   double p21 = 0.0001189;
   double p12 = 0.0001038;
   double p03 = 0.000226;
   double p40 = 5.659e-06;
   double p31 = 4.071e-06;
   double p22 = 6.077e-06;
   double p13 = 2.604e-06;
   double p04 = -2.154e-06;

   // roll coefficients

   double r00 = -8.671e-06;
   double r10 = 1.019;
   double r01 = -1.007;
   double r20 = 2.939e-05;
   double r11 = -1.657e-05;
   double r02 = 1.088e-05;
   double r30 = 4.714e-05;
   double r21 = -0.0001397;
   double r12 = 0.0001287;
   double r03 = -3.563e-05;
   double r40 = -9.05e-08;
   double r31 = -4.625e-07;
   double r22 = 2.809e-06;
   double r13 = -3.96e-06;
   double r04 = 1.643e-06;


   double roll_rad = toRadian(r00 + r10*rollxpiston + r01*pitchypiston + r20*pow(rollxpiston,2) + r11*rollxpiston*pitchypiston + r02*pow(pitchypiston,2) + r30*pow(rollxpiston,3) + r21*pow(rollxpiston,2)*pitchypiston + r12*rollxpiston*pow(pitchypiston,2) + r03*pow(pitchypiston,3) + r40*pow(rollxpiston,4) + r31*pow(rollxpiston,3)*pitchypiston + r22*pow(rollxpiston,2)*pow(pitchypiston,2) + r13*rollxpiston*pow(pitchypiston,3) + r04*pow(pitchypiston,4));


   double pitch_rad = -toRadian(p00 + p10*rollxpiston + p01*pitchypiston + p20*pow(rollxpiston,2) + p11*rollxpiston*pitchypiston + p02*pow(pitchypiston,2) + p30*pow(rollxpiston,3) + p21*pow(rollxpiston,2)*pitchypiston + p12*rollxpiston*pow(pitchypiston,2) + p03*pow(pitchypiston,3) + p40*pow(rollxpiston,4) + p31*pow(rollxpiston,3)*pitchypiston + p22*pow(rollxpiston,2)*pow(pitchypiston,2) + p13*rollxpiston*pow(pitchypiston,3) + p04*pow(pitchypiston,4));

  return std::make_pair (roll_rad,pitch_rad);
}

std::pair<double, double> Arm_IK::wristLinear2RadianVelocity(std::pair<double, double> roll_pitch_vel_in_mm){
  // TODO We assume that the relation between linear velocity and angular velocity ara same
  return roll_pitch_vel_in_mm;
}

double Arm_IK::elbowLinear2Radian_Position(double pos_in_mm){
  double x = pos_in_mm;
  double p1 = 1.905e-8;
  double p2 = -4.209e-6;
  double p3 = 0.0004141;
  double p4 = -0.02113;
  double p5 = 1.572;
  double p6 = 9.992;
  double degree_pos = p1*pow(x,5) + p2*pow(x,4) + p3*pow(x,3) + p4*pow(x,2) + p5*x + p6;
  // To solve URDF Beckhoff mismatch
  double beckhoff_urdf_error = 90.0; // 90 degree disparity
  degree_pos -= beckhoff_urdf_error;
  return toRadian(degree_pos);
}

double Arm_IK::elbowLinear2Radian_Velocity(double vel_in_mm){
  double x = vel_in_mm;
  double p1 = 1.905e-8;
  double p2 = -4.209e-6;
  double p3 = 0.0004141;
  double p4 = -0.02113;
  double p5 = 1.572;
  double vel_in_degree = 5*p1*pow(x,4) + 4*p2*pow(x,3) + 3*p3*pow(x,2) + 2*p4*x + p5;
  return vel_in_degree;
}

double Arm_IK::elbowRadian2Linear_Position(double pos_in_rad){

  double o1 = 28;
  double o2 = 39;
  double y = 226;
  double r = 55;

  double Lx = r*cos(toRadian(120) + pos_in_rad) + o2;
  double Ly = y - r*sin(toRadian(120) + pos_in_rad);
  double mm_distance = sqrt(pow(Lx,2) + pow(Ly,2) - pow(o1,2)) - 181.79;
  return mm_distance;
}

double Arm_IK::elbowRadian2Linear_Velocity(double pos_in_rad, double vel_in_rad){
  double o1 = 28;
  double o2 = 39;
  double y = 226;
  double r = 55;

  double Lx = r*cos(toRadian(120) + pos_in_rad) + o2;
  double Lx_dot = -r*sin(toRadian(120) + pos_in_rad)*vel_in_rad;
  double Ly = y - r*sin(toRadian(120) + pos_in_rad);
  double Ly_dot = -r*cos(toRadian(120) + pos_in_rad)*vel_in_rad;

  double numerator = Lx*Lx_dot + Ly*Ly_dot;
  double denominator = sqrt(pow(Lx,2) + pow(Ly,2) - pow(o1,2));
  return numerator/denominator;
}





























