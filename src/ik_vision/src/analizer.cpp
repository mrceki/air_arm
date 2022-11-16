#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

float toDegree(float radian){
    return (radian / M_PI) * 180.0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "analizer");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster br;

  ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("qr_pose", 1000);

  ros::Rate rate(10.0);

  geometry_msgs::TransformStamped transformStamped;

 // GET QR CODE INITIAL POSITION
  while (transformStamped.header.frame_id.empty()){

    try{
      transformStamped = tfBuffer.lookupTransform("world", "charuco",
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

  while (nh.ok()){
    reference_qr_tf.header.stamp = ros::Time::now();
    br.sendTransform(reference_qr_tf);

    geometry_msgs::TransformStamped result_tf;

    try{
      result_tf = tfBuffer.lookupTransform("qr_reference", "charuco",
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

    ROS_INFO_STREAM(   "x: " << result_tf.transform.translation.x << std::endl
                    << "y: " << result_tf.transform.translation.y << std::endl
                    << "z: " << result_tf.transform.translation.z << std::endl
                    << "r: " << toDegree(roll) << std::endl
                    << "p: " << toDegree(pitch) << std::endl
                    << "y: " << toDegree(yaw));

//// PUBLISH RESULTS AS ROSTOPIC

    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = result_tf.transform.translation.x;
    msg.twist.linear.y = result_tf.transform.translation.y;
    msg.twist.linear.z = result_tf.transform.translation.z;
    msg.twist.angular.x = toDegree(roll);
    msg.twist.angular.y = toDegree(pitch);
    msg.twist.angular.z = toDegree(yaw);

    pub.publish(msg);

    rate.sleep();
  }

  return 0;
}
