// this node is the controller of the inverted pendulum
// control input are position and velocity of pendulum angle and vehicle position
// control output is the torque exerted on the wheels of the vehicle

// communication includes:
    // listen to topics: "pendulum_angle", "vehicle_position"
    // apply joint effort by call gazebo service: "/gazebo/apply_joint_effort"
    // services for parameter tuning: "pendulum_angle_tuning", "vehicle_position_tuning"


#include <ros/ros.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <inverted_pendulum/pendulum_angle.h>
#include <inverted_pendulum/vehicle_position.h>
#include <inverted_pendulum/control_tuning_PD.h>
#include <inverted_pendulum/control_tuning_PID.h>

// control parameters
double g_pendulum_kp;
double g_pendulum_kd;
double g_vehicle_kp;
double g_vehicle_kd;

// robot state as inputs
inverted_pendulum::pendulum_angle g_pendulum_angle;
inverted_pendulum::vehicle_position g_vehicle_position;

// topic subscriber callback
void pendulumAngleCallback(const inverted_pendulum::pendulum_angle& message_holder) {
    g_pendulum_angle = message_holder;
}

// topic subscriber callback
void vehiclePositionCallback(const inverted_pendulum::vehicle_position& message_holder) {
    g_vehicle_position = message_holder;
}

// service server callback
bool pendulumAngleTuningCallback(inverted_pendulum::control_tuning_PDRequest& request,
    inverted_pendulum::control_tuning_PDResponse& response) {
    ROS_INFO("pendulum_angle_tuning callback activated");
    g_pendulum_kp = request.kp;
    g_pendulum_kd = request.kd;
    response.setting_is_done = true;
    return true;
}

// service server callback
bool vehiclePositionTuningCallback(inverted_pendulum::control_tuning_PDRequest& request,
    inverted_pendulum::control_tuning_PDResponse& response) {
    ROS_INFO("pendulum_angle_tuning callback activated");
    g_vehicle_kp = request.kp;
    g_vehicle_kd = request.kd;
    response.setting_is_done = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "inverted_pendulum_controller");
    ros::NodeHandle nh;

    // initialize subscribers for topics "pendulum_angle" and "vehicle_position"
    ros::Subscriber pendulum_angle_subscriber =
        nh.subscribe("pendulum_angle", 1, pendulumAngleCallback);
    ros::Subscriber vehicle_position_subscriber =
        nh.subscribe("vehicle_position", 1, vehiclePositionCallback);

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

