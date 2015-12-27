// this node is the controller of the inverted pendulum
// control input are position and velocity of pendulum angle and vehicle position
// control output is the torque exerted on the wheels of the vehicle

// communication includes:
    // listen to topics: "pendulum_angle", "vehicle_position"
    // apply joint effort by call gazebo service: "/gazebo/apply_joint_effort"
    // services for parameter tuning: "pendulum_angle_tuning", "vehicle_position_tuning"


#include <ros/ros.h>
#include <math.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <inverted_pendulum/pendulum_angle.h>
#include <inverted_pendulum/vehicle_position.h>
#include <inverted_pendulum/control_tuning_PD.h>
#include <inverted_pendulum/control_tuning_PID.h>

const double control_frequency = 100.0;

// control parameters
double g_pendulum_kp = 30.0;
double g_pendulum_kd = 1.0;
double g_vehicle_kp = 0.0;
double g_vehicle_kd = 0.0;

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

    // initialize a service client to apply vehicle traction
    ros::ServiceClient apply_vehicle_traction_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>(
        "/gazebo/apply_joint_effort");
    gazebo_msgs::ApplyJointEffort apply_vehicle_traction_srv_msg;
    apply_vehicle_traction_srv_msg.request.joint_name = "vehicle_joint";
    ros::Duration control_duration(1/control_frequency);
    apply_vehicle_traction_srv_msg.request.duration = control_duration;

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
    // these service will be called directly from terminal, no extra client node
    ros::ServiceServer pendulum_angle_tuning_service = 
        nh.advertiseService("pendulum_angle_tuning", pendulumAngleTuningCallback);
    ros::ServiceServer vehicle_position_tuning_service = 
        nh.advertiseService("vehicle_position_tuning", vehiclePositionTuningCallback);

    ros::Rate naptime(control_frequency);
    double traction_pendulum_angle = 0.0;
    double traction_vehicle_position = 0.0;
    double traction_output;  // control output as traction to the vehicle
    // control loop
    while (ros::ok()) {
        // check pendulum angle, exit if it's too large fails
        if (abs(g_pendulum_angle.position) > M_PI/6) {
            // exceed the critical point
            ROS_WARN("control fails, exceeds critical pendulum angle");
            break;
        }

        // ROS_INFO_STREAM("pendulum_angle: " << g_pendulum_angle.position);
        traction_pendulum_angle = g_pendulum_kp * g_pendulum_angle.position +
            g_pendulum_kd * g_pendulum_angle.velocity;
        traction_vehicle_position = g_vehicle_kp * (0 - g_vehicle_position.position) -
            g_vehicle_kd * g_vehicle_position.velocity;
        traction_output = traction_pendulum_angle + traction_vehicle_position;
        ROS_INFO_STREAM("traction_output: " << traction_output);
        // conversion from traction to torque exerted on four wheels
        apply_vehicle_traction_srv_msg.request.effort = traction_output;
        // send out srv msg
        apply_vehicle_traction_client.call(apply_vehicle_traction_srv_msg);

        // update global data
        ros::spinOnce();
        naptime.sleep();
    }
    return 0;
}

