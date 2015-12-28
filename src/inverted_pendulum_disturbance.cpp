// introduce distrubance to the pendulum joint, this is part of the control algorithm test

#include <ros/ros.h>
#include <math.h>
#include <gazebo_msgs/ApplyJointEffort.h>

// disturbance as sine
double g_ampl = 0.1;
double g_frep = 139;  // set a prime number here
double sample_dt = 0.001;

int main(int argc, char** argv) {
    ros::init(argc, argv, "inverted_pendulum_disturbance");
    ros::NodeHandle nh;

    // initialize a service client to apply the disturbance
    ros::ServiceClient apply_pendulum_disturbance_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>(
        "/gazebo/apply_joint_effort");
    gazebo_msgs::ApplyJointEffort apply_pendulum_disturbance_srv_msg;
    apply_pendulum_disturbance_srv_msg.request.joint_name = "pendulum_joint";
    ros::Duration disturbance_duration(sample_dt);
    apply_pendulum_disturbance_srv_msg.request.duration = disturbance_duration;

    // make sure apply_joint_effort service is ready
    bool service_ready = false;
    service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
    while (!service_ready) {
        ROS_INFO("waiting for /gazebo/apply_joint_effort service");
        ros::Duration(0.5).sleep();
        service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
    }
    ROS_INFO("/gazebo/apply_joint_effort service is ready");

    double timecount = 0.0;
    ros::Rate naptime(1/sample_dt);
    double output;
    double stop_time = 1.0;
    // publish loop
    while (ros::ok()) {
        if (timecount > stop_time) {
            break;  // stop output, disturbance act as start trigger
        }

        // call the service
        output = g_ampl * sin(2 * M_PI * g_frep * timecount);
        apply_pendulum_disturbance_srv_msg.request.effort = output;
        apply_pendulum_disturbance_client.call(apply_pendulum_disturbance_srv_msg);
        // ROS_INFO_STREAM("pendulum_joint_disturbance: " << output);

        // iterate timecount
        timecount = timecount + sample_dt;
        naptime.sleep();
    }
    return 0;
}

