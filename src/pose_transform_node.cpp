#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::PoseStamped initial_pose, initial_pose_world;
tf2::Transform world_to_map, map_to_world;
bool initial_pose_received = false;

// Publisher for the relative map pose
ros::Publisher relative_map_pose_pub;

// Callback function to receive the pose in the /mavros/vision_pose/pose_raw topic
void visionPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!initial_pose_received)
    {
        // Store the initial pose from the first received message in the /world frame
        initial_pose = *msg;
        world_to_map.setOrigin(tf2::Vector3(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z)); // Translation
        tf2::Quaternion quaternion(initial_pose.pose.orientation.x, initial_pose.pose.orientation.y, initial_pose.pose.orientation.z, initial_pose.pose.orientation.w);
        world_to_map.setRotation(quaternion);
        map_to_world = world_to_map.inverse();
        initial_pose_received = true;
    }

    try
    {   
        geometry_msgs::PoseStamped world_pose = *msg;
        geometry_msgs::PoseStamped relative_pose;

        // Extract the pose components
        tf2::Vector3 translation;
        tf2::Quaternion rotation;
        tf2::fromMsg(world_pose.pose.position, translation);
        tf2::fromMsg(world_pose.pose.orientation, rotation);

        // Apply the original transformation
        translation = map_to_world * translation;
        rotation = map_to_world * rotation;

        // Convert back to geometry_msgs types
        relative_pose.pose.position.x = translation.x();
        relative_pose.pose.position.y = translation.y();
        relative_pose.pose.position.z = translation.z();
        relative_pose.pose.orientation.x = rotation.x();
        relative_pose.pose.orientation.y = rotation.y();
        relative_pose.pose.orientation.z = rotation.z();
        relative_pose.pose.orientation.w = rotation.w();


        relative_pose.header.frame_id = "map";
        relative_pose.header.stamp = ros::Time::now();
        relative_map_pose_pub.publish(relative_pose);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Transform exception: %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_subtraction_node");
    ros::NodeHandle nh;

    // Subscribe to the topic that provides the pose in the /mavros/vision_pose/pose_raw topic
    ros::Subscriber vision_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose_raw", 1, visionPoseCallback);

    // Advertise the new topic for the relative map pose
    relative_map_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/vision_pose/pose", 1);

    ros::spin();

    return 0;
}