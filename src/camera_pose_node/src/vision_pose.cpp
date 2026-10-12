/**
 * @file vision_pose.cpp
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <time.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class vision_pose
{
public:
    vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_);

    double pi;
    struct attitude
    {
        double pitch;
        double roll;
        double yaw;
    };
    enum ESTIMATED_POSE_SOURCE
    {
        REALSENSE_T265,
        OPEN_VINS
    } pose_source_;

    attitude vrpnAttitude;
    attitude estimatedAttitude;
    attitude px4Attitude;

    geometry_msgs::PoseStamped vrpnPose;
    geometry_msgs::PoseStamped px4Pose;
    geometry_msgs::PoseStamped estimatedPose;

    bool vrpnPoseRec_flag;
    bool realsenseBridgePoseRec_flag;
    bool ovPoseRec_flag;
    bool vrpnPose_initilized;
    bool only_vrpn;
    bool estimatedOdomRec_flag;

    ros::Rate *rate;
    ros::Time start_time;
    double setupTime;
    double errx_const, erry_const, errz_const;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    std::string uavName;
    //估计位姿来源
    std::string pose_estimate_source_;

    ros::Subscriber vrpn_pose_sub;
    ros::Subscriber px4Pose_sub;
    ros::Subscriber openvins_pose_sub;
    ros::Publisher vision_pose_pub;
    ros::Subscriber odom_sub;

    /* 2026-10-12 增加：
     * frame_id_     : 发布出去的位姿使用的 frame_id；留空表示沿用源里程计消息里的 frame_id
     * odom_timeout_ : 输入里程计的超时时间(秒)；超时后停止发布，
     *                 避免把冻结的位姿当成有效视觉位姿继续喂给飞控
     * publish_rate  : 发布频率(Hz)
     */
    std::string frame_id_;
    double odom_timeout_;
    double publish_rate;
    ros::Time last_odom_recv_;

    void vrpn_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void openvins_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void estimator_odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void start();

};
/* 构造函数 */
vision_pose::vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_)
:nh(nh_), nh_private(nh_private_),pose_source_(OPEN_VINS),vrpnPose_initilized(false),pose_estimate_source_("open_vins")
{

    nh_private.param<std::string>("uavname", uavName, "px4_uav1");
    nh_private.param<std::string>("pose_estimate_source",pose_estimate_source_,"open_vins");
    nh_private.param<bool>("only_vrpn", only_vrpn,"false");
    // 2026-10-12 增加：发布频率 / frame_id / 输入超时，均可由 launch 配置
    nh_private.param<double>("publish_rate", publish_rate, 30.0);
    nh_private.param<std::string>("frame_id", frame_id_, "");
    nh_private.param<double>("odom_timeout", odom_timeout_, 0.5);

    if (pose_estimate_source_=="t265") pose_source_=REALSENSE_T265;

    pi = 3.1415926;
    setupTime = 10;
    rate = new ros::Rate(publish_rate);
    last_odom_recv_ = ros::Time(0);


   // vrpn_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/" + uavName + "/pose", 1, &vision_pose::vrpn_pose_cb,this);
    // openvins_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/ov_msckf/poseimu", 1, &vision_pose::openvins_pose_cb,this);
    px4Pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10,&vision_pose::px4Pose_cb,this);
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
    // odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 2, &vision_pose::estimator_odom_cb,this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 2, &vision_pose::estimator_odom_cb,this);
    realsenseBridgePoseRec_flag = false;
    vrpnPoseRec_flag = false;
    ovPoseRec_flag = false;
    only_vrpn = true;
    estimatedOdomRec_flag = false;

    estimatedAttitude.pitch = 0;
    estimatedAttitude.roll = 0;
    estimatedAttitude.yaw = 0;

    vrpnAttitude.pitch = 0;
    vrpnAttitude.roll = 0;
    vrpnAttitude.yaw = 0;

    px4Attitude.pitch = 0;
    px4Attitude.roll = 0;
    px4Attitude.yaw = 0;   
}

void vision_pose::estimator_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    estimatedPose.pose = msg->pose.pose;

    /* 2026-10-12 增加：原实现只拷贝了 pose.pose，header 从未赋值（frame_id 一直是空的）。
     * 这里补上：优先用 launch 参数 frame_id，未配置时沿用源里程计消息的 frame_id。 */
    estimatedPose.header.frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    estimatedAttitude.pitch = pitch * 180 / pi;
    estimatedAttitude.roll = roll * 180 / pi;
    estimatedAttitude.yaw = yaw * 180 / pi;

    estimatedOdomRec_flag = true;
    last_odom_recv_ = ros::Time::now();   // 2026-10-12：记录收到时刻，供超时判断
}


void vision_pose::px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    px4Pose.pose = msg->pose;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    px4Attitude.pitch = pitch * 180 / pi;
    px4Attitude.roll = roll * 180 / pi;
    px4Attitude.yaw = yaw * 180 / pi;
}

/*通过vrpn接受bebop位置消息*/
void vision_pose::vrpn_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    /*Motive通过 VRPN 发布的位置消息 单位是 米
    */
    vrpnPose.pose = msg->pose;
    estimatedPose = *msg;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    vrpnAttitude.pitch = pitch * 180 / pi;
    vrpnAttitude.roll = roll * 180 / pi;
    vrpnAttitude.yaw = yaw * 180 / pi;

    vrpnPoseRec_flag = true;
}


// void vision_pose::openvins_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
// {
//     estimatedPose.pose.position.x = msg->pose.pose.position.y;
//     estimatedPose.pose.position.y = -msg->pose.pose.position.x;
//     estimatedPose.pose.position.z = msg->pose.pose.position.z;
//     tf2::Quaternion q_ov, q_rot_r, q_rot_y, q_est, q_gamma;
//     tf2::fromMsg(msg->pose.pose.orientation,q_ov);
//     // double r = 1.5707963, p = 0, y = 0; //Rotate the previous pose by 90degree about Z
//     q_rot_r.setRPY(1.5707963,0,0);
//     q_rot_y.setRPY(0,0,1.5707963);
//     q_gamma.setRPY(0,0,-1.5707963);
//     q_est = q_gamma * q_ov * q_rot_r * q_rot_y;
//     q_est.normalize();
//     tf2::convert(q_est,estimatedPose.pose.orientation);   
//     double roll_ov, pitch_ov, yaw_ov;
//     tf2::Matrix3x3(q_est).getRPY(roll_ov, pitch_ov, yaw_ov);
//     estimatedAttitude.pitch = pitch_ov * 180 / pi;
//     estimatedAttitude.roll = roll_ov * 180 / pi;
//     estimatedAttitude.yaw = yaw_ov * 180 /pi;
//     //cout<<"pitch: "<<pitch_ov*180/pi<<" roll: "<<roll_ov*180/pi<<" yaw: "<<yaw_ov*180/pi<<endl;
//     ovPoseRec_flag = true;
// }

void vision_pose::start()
{
    while(ros::ok())
    {
     	
        if(estimatedOdomRec_flag == false){
            cout << "\033[K" << "\033[31m estimatedPose no receive!!! \033[0m" << endl;
	    } else if (odom_timeout_ > 0.0 &&
	               (ros::Time::now() - last_odom_recv_).toSec() > odom_timeout_) {
            /* 2026-10-12 增加：输入里程计中断超过 odom_timeout_ 秒后停止发布。
             * 否则节点会把最后一帧位姿配上新鲜的时间戳反复发出去，
             * 飞控会把这份"冻结"的视觉位姿当成有效观测继续融合。 */
            ROS_WARN_THROTTLE(2.0, "odometry stale for %.2fs, vision pose NOT published",
                              (ros::Time::now() - last_odom_recv_).toSec());
        } else {
            estimatedPose.header.stamp = ros::Time::now();
	    vision_pose_pub.publish(estimatedPose);
            

        }
        ros::spinOnce();
        rate->sleep();
    }
    // cout << "\033[2J" << endl;
    cout << "\033[9B" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_pose_node");
    ros::NodeHandle nh_("");
    ros::NodeHandle nh_private_("~");
    vision_pose vision(nh_,nh_private_);
    vision.start();

    return 0;
}
