#include <bluerov_control/bluerov_subs.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

void go() {
    ros::NodeHandle nh;
    ros::Rate rate(1);
    ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);

    Subscriber sub(nh);        
    
    // step 1 - connect
    sub.connect();
    geometry_msgs::PoseStamped  poseTarget;

    // step 2 - pose update (can do in while loop as well)
    poseTarget.pose.position.x = 10;
    poseTarget.pose.position.y = 10;
    poseTarget.pose.position.z = -2;
    
    while(ros::ok()){
        
        // step 3 - move to guided mode & arm throttle
        sub.guided_armed();

        posePub.publish(poseTarget);        
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "subscriber");
    go();
}