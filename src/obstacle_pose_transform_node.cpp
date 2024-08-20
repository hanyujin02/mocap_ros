#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>

geometry_msgs::PoseStamped initial_pose, initial_pose_world;
tf2::Transform world_to_map, map_to_world;
bool initial_pose_received = false;

nav_msgs::Path poseVec;
// Publisher for the relative map pose
ros::Publisher obstacle_relative_map_pose_pub;
std::thread pubWorker;
std::vector<std::string> topics;
int numOb;
bool newPoseReceived = false;


// Publisher for the relative map pose
ros::Publisher relative_map_pose_pub;
ros::Publisher relative_map_obstacle_pose_pub;

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

// Callback function to receive the pose in the /mavros/vision_pose/pose_raw topic
void obstaclePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& topicName)
{
    if (!initial_pose_received)
    {
        return;
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
        int obIdx;
        for (int i=0;i<int(topics.size());i++){
            if (topicName == topics[i]){
                obIdx = i;
            }
        }
        poseVec.poses[obIdx] = relative_pose;


        // relative_map_pose_pub.publish(relative_pose);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Transform exception: %s", ex.what());
    }
}
void pubCB(void){
    ros::Rate r(30);
    while (ros::ok()){
        poseVec.header.frame_id = "map";
        obstacle_relative_map_pose_pub.publish(poseVec);
        r.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_subtraction_node");
    ros::NodeHandle nh;
    nh.getParam("/mocap/topic_name", topics);
    numOb = topics.size();

    // Subscribe to the topic that provides the pose in the /mavros/vision_pose/pose_raw topic
    ros::Subscriber vision_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose_raw", 1, visionPoseCallback);

    // Advertise the new topic for the relative map pose
    relative_map_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/vision_pose/pose", 1);

    std::vector<ros::Subscriber> model_state_sub;
    poseVec.poses.resize(numOb);
    for (const auto& topic : topics){
        // Subscribe to the topic that provides the pose in the /mavros/vision_pose/pose_raw topic
        ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(
            topic, 1, [topic](const geometry_msgs::PoseStamped::ConstPtr& msg) { obstaclePoseCallback(msg, topic); });
        model_state_sub.push_back(sub);
        
    }
    // Advertise the new topic for the relative map pose
    obstacle_relative_map_pose_pub = nh.advertise<nav_msgs::Path>(
        "/mocap/model_states", 1);
    pubWorker = std::thread(&pubCB);
    ros::spin();

    return 0;
}