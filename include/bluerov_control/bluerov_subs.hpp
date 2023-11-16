
/*
######################################### Message Types #############################################################  
https://github.com/patrickelectric/bluerov_ros_playground/blob/master/doc/topics_and_data.md
#####################################################################################################################
*/

/*
######################################### Mavros Services ###########################################################
    /mavros/set_mode   - set flight mode of the controller. Most often used to set the GUIDED mode to accept commands.
    /mavros/cmd/arming - arm or disarm drone motors (change arming status).
#####################################################################################################################
*/

/*
######################################### Mavros Subscribers ########################################################
    /diagnostics                        - connection status and bot information.
    /mavros/global_position/local       - the global position in the UTM coordinate system.
    /mavros/global_position/velocity    - current speed in local coordinates and angular velocities.
    /mavros/global_position/global      - current global position (latitude, longitude, altitude).
    /mavros/local_position/pose         - local position and orientation of the copter in the ENU coordinate system.
    /mavros/local_position/odom         - local position, orientation and vel of the copter in the ENU coordinate system.
    /mavros/global_position/rel_alt     - relative altitude (relative to the arming altitude).
    /mavros/state                       - status of connection to the flight controller and flight controller mode.
#####################################################################################################################
*/

/*
######################################### Mavros Publishers #########################################################
    /mavros/setpoint_position/local   - set target position and yaw of the drone (in the ENU coordinate system).
    /mavros/setpoint_position/global  - set target position in global coordinates (latitude, longitude, altitude) 
                                        and yaw of the drone.
    /mavros/setpoint_velocity/cmd_vel - set target linear velocity of the drone.
    /mavros/setpoint_attitude/attitude & /mavros/setpoint_attitude/att_throttle - set target attitude and 
                                                                                  throttle level.
    /mavros/setpoint_attitude/cmd_vel & /mavros/setpoint_attitude/att_throttle  - set target angular velocity 
                                                                                  and throttle level.
#####################################################################################################################
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

class Subscriber {
    public:
        nav_msgs::Odometry          currentOdom;
        mavros_msgs::State          currentState;
        Subscriber(ros::NodeHandle nh) {
            odomSub         = nh.subscribe("/mavros/local_position/odom", 1, &Subscriber::odomCallback, this);
            stateSub        = nh.subscribe("/mavros/state", 1, &Subscriber::stateCallback, this);
            arming_client   = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
            offb_set_mode.request.custom_mode     = "GUIDED";
            arm_cmd.request.value                 = true;
        }
        void guided_armed() {
            if( currentState.mode != "GUIDED" ){
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                    ROS_INFO("Guided mode enabled");
                }
            } else {
                if( !currentState.armed ){
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                }
            }
        }
        void connect() {
            while(ros::ok() && !currentState.connected){
                ros::spinOnce();
            }
        }
    private:
        ros::Subscriber          stateSub;
        ros::Subscriber          odomSub;
        ros::ServiceClient       arming_client;
        ros::ServiceClient       set_mode_client;
        mavros_msgs::SetMode     offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        void stateCallback(const mavros_msgs::State::ConstPtr &msg) {
            currentState = *msg;
        }
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
            currentOdom = *msg;
            std::cout << "pose: " << currentOdom.pose.pose.position.x << ", " << currentOdom.pose.pose.position.y << ", " << currentOdom.pose.pose.position.z << std::endl;
        }
};


