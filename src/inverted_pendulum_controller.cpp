// this node is the controller of the inverted pendulum
// control output is the torque exerted on the wheels

#include <ros/ros.h>
#include <gazebo_msgs/ApplyJointEffort.h>

double g_upright_kp;
double g_upright_kv;

bool pendulumParameterCallback()

int main(int argc, char** argv) {
    ros::init(argc, argv, "inverted_pendulum_controller");
    ros::NodeHandle nh;

    // initialize a service client to apply joint effort
    ros::ServiceClient apply_joint_effort_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>(
        "/gazebo/apply_joint_effort");
    gazebo_msgs::ApplyJointEffort apply_joint_effort_srv_msg;

    // make sure apply_joint_effort service is ready
    bool service_ready = false;
    service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
    while (!service_ready) {
        ROS_INFO("waiting for /gazebo/apply_joint_effort service");
        ros::Duration(0.5).sleep();
        service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
    }
    ROS_INFO("/gazebo/apply_joint_effort service is ready");

    // initialize a service server to tune all the parameters in the controller
    ros::ServiceServer pendulum_parameter_service = 
        nh.advertiseService("pendulum_parameter", pendulumParameterCallback);




// check rotation angle in the control loop, exit if control fails


}

