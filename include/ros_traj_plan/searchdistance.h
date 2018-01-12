#pragma once
#ifndef _SEARCHDISTANCE_H
#define _SEARCHDISTANCE_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "geometry_msgs/PoseStamped.h" 
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "auto_flight/destinate.h"
#include "mavros_msgs/CommandBool.h"  //service message
#include "mavros_msgs/SetMode.h"     //service message
#include "mavros_msgs/State.h"         //topic message

#include "ros_traj_plan/uavtrans.h"
#include "ros_traj_plan/Trajectory.hpp"
#include "ros_traj_plan/TrajectoryLibrary.hpp"



class Uav_trajplannimg
{
	public:
	Uav_trajplannimg();

	uavTrans get_trans_msg() const;
	geometry_msgs::PoseStamped get_trans_msg_a() const;

	geometry_msgs::PoseStamped get_destinate_coor() const;
	int get_get_des_() const;

	pcl::PointCloud<pcl::PointXYZ> get_cloudmap() const;

	void pub_waypoint(geometry_msgs::PoseStamped way_point__);

	bool arrive_destination(geometry_msgs::PoseStamped trans_msg_a,geometry_msgs::PoseStamped des_coordinate) const;

	//终点读取轨迹库初始化
	void fly_init();
	void fly_takeoff(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* octree_map_);
	bool fly_traj_plan(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* octree_map_);
	
	private:

	void call_transmsg(const geometry_msgs::PoseStamped& msg);
	void map_callback(const sensor_msgs::PointCloud2& map_msg);
	void state_Callback(const mavros_msgs::State::ConstPtr& msg);
	bool res_for_des(auto_flight::destinate::Request& req,auto_flight::destinate::Response& res);

	ros::NodeHandle n1;
	ros::Publisher waypoint_pub;
	ros::Publisher true_way_pub;
	ros::Publisher plan_way_pub;
	ros::Publisher map_pub;
   
    //订阅飞机位姿
	ros::Subscriber trans_sub;
	//订阅点云地图
	ros::Subscriber map_sub;
	//creat subscriber  订阅飞控的状态【连接状态／解锁状态／外部控制的标志】
    ros::Subscriber state_sub;
    
	//终点服务器端
	ros::ServiceServer des_ser;
	//飞机解锁相关服务
    ros::ServiceClient arming_client ;
    ros::ServiceClient set_mode_client;
	
	//订阅飞机姿态数据
	uavTrans trans_msg;
	geometry_msgs::PoseStamped trans_msg_a;
	mavros_msgs::State current_state;
    //发布航点与轨迹
	geometry_msgs::PoseStamped way_point_;
	nav_msgs::Path gui_way;
	nav_msgs::Path way_point_path;
    //点云地图信息
	sensor_msgs::PointCloud2 output;
	pcl::PointCloud<pcl::PointXYZ> cloud;
    //终点信息
	geometry_msgs::PoseStamped des_coordinate;
	int get_des_;

    TrajectoryLibrary TrajLibrarymain; //地面安全距离
	double point_distances;
	double closest_point_distances;
	double current_point_distance;
	double closest_current_point_distance;
	float current_position[3];
	double safe_dis;

	clock_t start, finish;
	
	double totaltime;
	double t_last;
	double thisseartime;

	int this_tra;
	int this_tra_last;
	double traj_closest_dist;
	Trajectory *farthest_traj = nullptr;
	std::vector<geometry_msgs::PoseStamped> way_point;
	int way_point_i;

	uavTrans trans_msg1;
	geometry_msgs::PoseStamped trans_msg_a1;
	geometry_msgs::PoseStamped des_coordinate_;
	double fly_yaw;
    std::tuple<int, double, Trajectory*,std::vector<geometry_msgs::PoseStamped>> chosetraj;
    
	static int straight_pre_traj_no[5] ;
	static int left_pre_traj_no[5];
	static int right_pre_traj_no[5];
	static int straightl_pre_traj[5] ;
	static int straightr_pre_traj[5];
	static int left_pre_traj[5] ;
	static int right_pre_traj[5];
	static int take_off_traj[5];

};



#endif // !_SEARCHDISTANCE_H


