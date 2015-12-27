// this node will get robot state from gazebo and publish them as topics
// "pendulum_angle", msg type "inverted_pendulum/pendulum_angle"
// "vehicle_position", msg type "inverted_pendulum/vehicle_position"

#include <ros/ros.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetLinkState.h>
#include <inverted_pendulum/pendulum_angle.h>
#include <inverted_pendulum/vehicle_position.h>

const double g_publish_frequency = 500.0;

int main(int argc, char** argv) {
    ros::init(argc, argv, "inverted_pendulum_publisher");
    ros::NodeHandle nh;

    // instantiate service clients to get vehicle position and pendulum angle
    ros::ServiceClient get_pendulum_angle_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
        "/gazebo/get_joint_properties");
    ros::ServiceClient get_vehicle_position_client = nh.serviceClient<gazebo_msgs::GetLinkState>(
        "/gazebo/get_link_state");
    gazebo_msgs::GetJointProperties get_pendulum_angle_srv_msg;
    get_pendulum_angle_srv_msg.request.joint_name = "pendulum_joint";
    gazebo_msgs::GetLinkState get_vehicle_position_srv_msg;
    get_vehicle_position_srv_msg.request.link_name = "vehicle_body";
    get_vehicle_position_srv_msg.request.reference_frame = "world";

    // make sure gazebo services are ready
    bool service_ready = false;
    // gazebo/get_joint_properties
    service_ready = ros::service::exists("/gazebo/get_joint_properties",true);
    while (!service_ready) {
        ROS_INFO("waiting for /gazebo/get_joint_properties service");
        ros::Duration(0.5).sleep();
        service_ready = ros::service::exists("/gazebo/get_joint_properties",true);
    }
    ROS_INFO("/gazebo/get_joint_properties service is ready");
    // gazebo/get_link_state
    service_ready = ros::service::exists("/gazebo/get_link_state",true);
    while (!service_ready) {
        ROS_INFO("waiting for /gazebo/get_link_state service");
        ros::Duration(0.5).sleep();
        service_ready = ros::service::exists("/gazebo/get_link_state",true);
    }
    ROS_INFO("/gazebo/get_link_state service is ready");

    // instantiate publishers with topics "pendulum_angle" and "vehicle_position"
    // both topic contains the position and velocity information
    ros::Publisher pendulum_angle_publisher = nh.advertise<inverted_pendulum::pendulum_angle>(
        "pendulum_angle", 1);
    ros::Publisher vehicle_position_publisher = nh.advertise<inverted_pendulum::vehicle_position>(
        "vehicle_position", 1);
    inverted_pendulum::pendulum_angle pendulum_angle_msg;
    inverted_pendulum::vehicle_position vehicle_position_msg;

    // publish loop
    ros::Rate naptime(g_publish_frequency);
    bool vehicle_position_srv_first_call = true;  // first time to call /gazebo/get_link_state
    double last_vehicle_position;
    double time_interval = 1/g_publish_frequency;
    while (ros::ok()) {
        // get pendulum angle
        get_pendulum_angle_client.call(get_pendulum_angle_srv_msg);
        pendulum_angle_msg.position = get_pendulum_angle_srv_msg.response.position[0];
        pendulum_angle_msg.velocity = get_pendulum_angle_srv_msg.response.rate[0];

        // get vehicle position
        get_vehicle_position_client.call(get_vehicle_position_srv_msg);
        vehicle_position_msg.position = 
            get_vehicle_position_srv_msg.response.link_state.pose.position.x;
        if (vehicle_position_srv_first_call) {
            vehicle_position_srv_first_call = false;
            // initialize the vehicle position for the first time, velocity will be zero
            last_vehicle_position = vehicle_position_msg.position;
        }
        vehicle_position_msg.velocity =
            (vehicle_position_msg.position - last_vehicle_position) / time_interval;
        last_vehicle_position = vehicle_position_msg.position;

        // publish pendulum angle and vehicle position
        pendulum_angle_publisher.publish(pendulum_angle_msg);
        // ROS_INFO_STREAM("pendulum_angle: " << pendulum_angle_msg.position);
        vehicle_position_publisher.publish(vehicle_position_msg);
        naptime.sleep();
    }
    return 0;
}

