#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

float toDegree(float radian){
    return (radian / M_PI) * 180.0;
}

void rotate(geometry_msgs::TransformStamped& reference_qr_tf, double direction){
  tf2::Quaternion q_orig, q_rot, q_new;

  // Get the original orientation of 'reference_qr_tf'
  tf2::convert(reference_qr_tf.transform.rotation , q_orig);

  double r=0, p=0, y=direction*1.57079633;  // Rotate the previous pose around X axis
  q_rot.setRPY(r, p, y);

  q_new = q_rot*q_orig;  // Calculate the new orientation
  q_new.normalize();

  // Stuff the new rotation back into the pose. This requires conversion into a msg type
  tf2::convert(q_new, reference_qr_tf.transform.rotation);
}

tf2::Quaternion toQuaternian(geometry_msgs::TransformStamped& transformStamped){
    tf2::Quaternion q(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w);
    return q;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "analizer");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster br;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("qr_pose", 1000);

  ros::Rate rate(10.0);

  geometry_msgs::TransformStamped transformStamped;

 // GET QR CODE INITIAL POSITION
  while (transformStamped.header.frame_id.empty()){

    try{
      transformStamped = tfBuffer.lookupTransform("world", "fiducial_175",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    ROS_INFO_STREAM("TF: " << transformStamped.transform);
    rate.sleep();
  }

 // PUBLISH QR CODE INITIAL POSITION AS A REFERENCE FRAME

  geometry_msgs::TransformStamped reference_qr_tf, ref_tf_inv;

  reference_qr_tf.header.stamp = ros::Time::now();
  reference_qr_tf.header.frame_id = "world";
  reference_qr_tf.child_frame_id = "qr_reference";
  reference_qr_tf.transform.translation.x = transformStamped.transform.translation.x;
  reference_qr_tf.transform.translation.y = transformStamped.transform.translation.y;
  reference_qr_tf.transform.translation.z = transformStamped.transform.translation.z;
  reference_qr_tf.transform.rotation.x = transformStamped.transform.rotation.x;
  reference_qr_tf.transform.rotation.y = transformStamped.transform.rotation.y;
  reference_qr_tf.transform.rotation.z = transformStamped.transform.rotation.z;
  reference_qr_tf.transform.rotation.w = transformStamped.transform.rotation.w;

  tf2::Quaternion q_orig, q_rot, q_new;
  tf2::convert(reference_qr_tf.transform.rotation , q_orig);
  // parallelize frame orientation to base/floor
  // q_rot matrix is calculated manually and it is constant
  double r=-0.013, p=-0.179, y=0;
  q_rot.setRPY(r, p, y);

  q_new = q_rot*q_orig;  // Calculate the new orientation
  q_new.normalize();
  tf2::convert(q_new, reference_qr_tf.transform.rotation);

  rotate(reference_qr_tf, 1); // Align qr coordinate frame with wrist coordinate frame

  while (nh.ok()){
    reference_qr_tf.header.stamp = ros::Time::now();
    br.sendTransform(reference_qr_tf);

    try{
      transformStamped = tfBuffer.lookupTransform("world", "fiducial_175",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    // parallelize frame orientation to base/floor
    tf2::Quaternion f_q = toQuaternian(transformStamped);
    f_q = q_rot*f_q;
    f_q.normalize();

    tf2::convert(f_q, transformStamped.transform.rotation);

// PUBLISH ACTIVE QR CODE REFERENCE FRAME

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "active_qr";
    br.sendTransform(transformStamped);

    geometry_msgs::TransformStamped result_tf;

    try{
      result_tf = tfBuffer.lookupTransform("qr_reference", "active_qr",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    tf2::Quaternion q(
        result_tf.transform.rotation.x,
        result_tf.transform.rotation.y,
        result_tf.transform.rotation.z,
        result_tf.transform.rotation.w);
    tf2::Matrix3x3 m(q);

// rotate result frame to align results in desired reference frame

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double tmp = roll;
    roll = pitch;
    pitch = -tmp;

    ROS_INFO_STREAM(   "x: " << result_tf.transform.translation.x << std::endl
                    << "y: " << result_tf.transform.translation.y << std::endl
                    << "z: " << result_tf.transform.translation.z << std::endl
                    << "r: " << toDegree(roll) << std::endl
                    << "p: " << toDegree(pitch) << std::endl
                    << "y: " << toDegree(yaw));

// PUBLISH RESULTS AS ROSTOPIC

    geometry_msgs::Twist msg;
    msg.linear.x  = result_tf.transform.translation.x;
    msg.linear.y  = result_tf.transform.translation.y;
    msg.linear.z  = result_tf.transform.translation.z;
    msg.angular.x = toDegree(roll);
    msg.angular.y = toDegree(pitch);
    msg.angular.z = toDegree(yaw);

    pub.publish(msg);

    rate.sleep();
  }

  return 0;
}
