#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Callback function to receive the pose in the world frame
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    try
    {
        // Create a TransformListener to listen for transformations
        static tf2_ros::Buffer tfBuffer;
        static tf2_ros::TransformListener tfListener(tfBuffer);

        // Create a PoseStamped message in the /world frame
        geometry_msgs::PoseStamped worldPose = *msg;
        worldPose.header.stamp = ros::Time(0); // Use the latest available transform
        worldPose.header.frame_id = "world"; // Set the frame to /world

        // Use tfBuffer to lookup the transformation from /world to /map
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("map", "world", ros::Time(0));

        // Transform the PoseStamped message from /world to /map
        geometry_msgs::PoseStamped mapPose;
        tf2::doTransform(worldPose, mapPose, transformStamped);

        // Create a Publisher to publish the transformed PoseStamped message
        static ros::NodeHandle nh;
        static ros::Publisher transformedPosePub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

        // Publish the transformed PoseStamped message
        transformedPosePub.publish(mapPose);

        // ROS_INFO("Transformed pose published!");
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Transform exception: %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_transform_node");
    ros::NodeHandle nh;

    // Subscribe to the topic that provides the pose in the world frame
    ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose_raw", 10, poseCallback);

    ros::spin();

    return 0;
}
