// this node is the controller of the inverted pendulum
// control output is the torque exerted on the wheels

#include <ros/ros.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <inverted_pendulum/control_tuning_PD.h>
#include <inverted_pendulum/control_tuning_PID.h>

double g_pendulum_kp;
double g_pendulum_kd;

double g_vehicle_kp;
double g_vehicle_kd;

bool pendulumAngleTuningCallback(inverted_pendulum::control_tuning_PDRequest& request,
    inverted_pendulum::control_tuning_PDResponse& response)
{
    ROS_INFO("pendulum_angle_tuning callback activated");
    g_pendulum_kp = 
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "inverted_pendulum_controller");
    ros::NodeHandle nh;

    // initialize a service client to apply wheel torque
    ros::ServiceClient apply_wheel_torque_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>(
        "/gazebo/apply_joint_effort");
    gazebo_msgs::ApplyJointEffort apply_wheel_torque_srv_msg;

    // make sure apply_joint_effort service is ready
    bool service_ready = false;
    service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
    while (!service_ready) {
        ROS_INFO("waiting for /gazebo/apply_joint_effort service");
        ros::Duration(0.5).sleep();
        service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
    }
    ROS_INFO("/gazebo/apply_joint_effort service is ready");

    // initialize service servers to tune control parameters
    ros::ServiceServer pendulum_angle_tuning_service = 
        nh.advertiseService("pendulum_angle_tuning", pendulumAngleTuningCallback);
    ros::ServiceServer vehicle_position_tuning_service = 
        nh.advertiseService("vehicle_position_tuning", vehiclePositionTuningCallback);



// check rotation angle in the control loop, exit if control fails


}

