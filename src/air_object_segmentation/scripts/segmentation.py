#!/usr/bin/env python

# Author: Berkay GOKCEN

# Import modules
from pcl_helper import *
from air_object_segmentation.srv import *
import tf
from geometry_msgs.msg import PoseStamped
# Callback function for your Point Cloud Subscriber
def pcl_callback(ros_msg):

    rospy.logdebug("Segmentation - massage arrived!")
    # Convert ROS msg to PCL data
    ################################
    pcl_msg = ros_to_pcl(ros_msg)
    
    # Voxel Grid Downsampling filter
    ################################
    # Create a VoxelGrid filter object for our input point cloud
    vox = pcl_msg.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    # Note: this (1) means 1mx1mx1m is a poor choice of leaf size   
    # Experiment and find the appropriate size!
    LEAF_SIZE = 0.02

    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    downsampled = vox.filter()

    # PassThrough filter
    ################################
    # Create a PassThrough filter object.
    passthrough = downsampled.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.2
    axis_max = 0.8
    passthrough.set_filter_limits(axis_min, axis_max)
    passed = passthrough.filter()

    # Limiting on the Y axis too to avoid having the bins recognized as snacks
    passthrough = passed.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.45
    axis_max = +0.45
    passthrough.set_filter_limits(axis_min, axis_max)
    passed = passthrough.filter()

    passed_ros = pcl_to_ros(passed)

    # Transform Pointcloud
    print("wait_for_service 1")
    rospy.wait_for_service('transform_pc_srv')
    print("wait_for_service 2")
    try:
        tf_function = rospy.ServiceProxy('transform_pc_srv', PcTransform)
        resp = tf_function(passed_ros)
        passed = ros_to_pcl(resp.response_pc)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    # brkygkcn
    # TODO: Check is pointcloud null
    # RANSAC plane segmentation
    ################################
    # Create the segmentation object
    seg = passed.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance
    # for segmenting the table
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers
    # pcd for table
    ################################
    cloud_table = passed.extract(inliers, negative=False)

    min_z, max_z = get_min_max_z(cloud_table)

    print("min: "+str(min_z)+", max: "+str(max_z))

    passthrough = passed.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = max_z
    axis_max = 999
    passthrough.set_filter_limits(axis_min, axis_max)
    passed = passthrough.filter()

    red_pc = color_filter(passed)
    downsampled = XYZRGB_to_XYZ(red_pc)
    # Outlier Removal Filter
    # Much like the previous filters, we start by creating a filter object:
    outlier_filter = downsampled.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(5)
    # Set threshold scale factor
    x = 0.1
    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)
    # Finally call the filter function for magic
    outliers_removed_xyz = outlier_filter.filter()
    color = [255, 0, 0]
    cloud_object = XYZ_to_XYZRGB(outliers_removed_xyz, color)

    target_pose = calculate_mean(outliers_removed_xyz)

#    pose = PoseStamped()
#    pose.header.frame_id = "base_link"
#    pose.header.stamp = rospy.Time.now()
#    pose.pose.position.x = target_pose[0]
#    pose.pose.position.y = target_pose[1]
#    pose.pose.position.z = target_pose[2]

#    target_pose_pub.publish(pose)

    br = tf.TransformBroadcaster()
    br.sendTransform((target_pose[0], target_pose[1], target_pose[2]),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "ball_tf",
                     "base_link")
    # Convert PCL data to ROS messages
    ################################
#    ros_outliers_removed = pcl_to_ros(outliers_removed_rgb)
    ros_passed = pcl_to_ros(passed)
    ros_cloud_objects = pcl_to_ros(cloud_object)
#    ros_cloud_table   = pcl_to_ros(cloud_table)
#    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
#    ros_selected_cloud = pcl_to_ros(selected_cloud)

    # Publish ROS messages
    ################################
#    pcl_outliers_removed_pub.publish(ros_outliers_removed)
    pcl_passed_pub.publish(ros_passed)
    pcl_objects_pub.publish(ros_cloud_objects)
#    pcl_table_pub.publish(ros_cloud_table)
#    pcl_cluster_pub.publish(ros_cluster_cloud)
#    pcl_selected_pub.publish(ros_selected_cloud)

    return

################################
if __name__ == '__main__':
    
    # ROS node initialization
    ################################
    rospy.init_node('air_object_segmentation', anonymous=True)
    sub_pcl_topic_name = rospy.get_param('~sub_pcl_topic_name', 'point_cloud_in')
    pub_pcl_topic_name = rospy.get_param('~pub_pcl_topic_name', 'grasp_object')

    # Create Subscribers
    ################################
    pcl_sub = rospy.Subscriber(sub_pcl_topic_name, pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    ################################
#    pcl_outliers_removed_pub = rospy.Publisher("/pcl_outliers_removed", PointCloud2, queue_size=1)
    pcl_passed_pub = rospy.Publisher("/pcl_passed", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
#    pcl_table_pub   = rospy.Publisher("/pcl_table"  , PointCloud2, queue_size=1)
    #pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    #pcl_selected_pub = rospy.Publisher(pub_pcl_topic_name, PointCloud2, queue_size=1)
#    target_pose_pub = rospy.Publisher('airarm/goal', PoseStamped, queue_size=10)
    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    ################################
    while not rospy.is_shutdown():
        rospy.spin()
