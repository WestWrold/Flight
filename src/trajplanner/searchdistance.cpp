#include"ros_traj_plan/searchdistance.h"
//#include "boost/shared_ptr.hpp"
#include "ros_traj_plan/Trajectory.hpp"
#include <iostream>

using namespace std;

Uav_trajplannimg::Uav_trajplannimg()
 { 
	get_des_=0;
	closest_point_distances=-1.0;
    closest_current_point_distance=-1.0;
	safe_dis=1.5;
	way_point_i=0;
	
	gui_way.header.frame_id="map";
	way_point_path.header.frame_id="map";
	trans_sub=n1.subscribe("/mavros/local_position/pose",1,&Uav_trajplannimg::call_transmsg,this);
	map_sub=n1.subscribe("pcl_output_map",1,&Uav_trajplannimg::map_callback,this);
	state_sub = n1.subscribe("mavros/state", 10, &Uav_trajplannimg::state_Callback,this);

	waypoint_pub=n1.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",1);
	true_way_pub=n1.advertise<nav_msgs::Path>("gui_way",1);
	plan_way_pub=n1.advertise<nav_msgs::Path>("way_point_path",1);
	map_pub = n1.advertise<sensor_msgs::PointCloud2> ("map_output", 1); 

	des_ser=n1.advertiseService("set_destination",&Uav_trajplannimg::res_for_des,this);
	arming_client = n1.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client=n1.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	 
}

     //int Uav_trajplannimg::straight_pre_traj_no[5] = {4,2,1,0,4};
	int Uav_trajplannimg::straight_pre_traj_no[5] = {4,4,4,4,4};
    int Uav_trajplannimg::left_pre_traj_no[5] ={4,0,4,4,4};
	int Uav_trajplannimg::right_pre_traj_no[5]={4,0,4,4,4};
	int Uav_trajplannimg::straightl_pre_traj[5] ={ 2,1,4,2,1};
	int Uav_trajplannimg::straightr_pre_traj[5] = {1,2,4,1,2};
	int Uav_trajplannimg::left_pre_traj[5] = { 0,1,1,0,0};
	int Uav_trajplannimg::right_pre_traj[5] = {0,2,2,0,0};
	int Uav_trajplannimg::take_off_traj[5]={3,3,3,3,3};

void Uav_trajplannimg::map_callback(const sensor_msgs::PointCloud2& map_msg)
{
	pcl::fromROSMsg(map_msg,cloud);
}

void Uav_trajplannimg::state_Callback(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state = *msg;
    }

void Uav_trajplannimg::call_transmsg(const geometry_msgs::PoseStamped& msg)
{
	trans_msg_a=msg;
	trans_msg.uav_position[0]=msg.pose.position.x;
	trans_msg.uav_position[1]=msg.pose.position.y;
	trans_msg.uav_position[2]=msg.pose.position.z;
	trans_msg.uav_quat[0]=msg.pose.orientation.w;
	trans_msg.uav_quat[1]=msg.pose.orientation.x,
	trans_msg.uav_quat[2]=msg.pose.orientation.y;
	trans_msg.uav_quat[3]=msg.pose.orientation.z;
	gui_way.poses.push_back(trans_msg_a);
	gui_way.header.frame_id="map";
	true_way_pub.publish(gui_way);
	
	/*
	pcl::toROSMsg(*current_cloud_, output); 
	output.header.frame_id = "map";
	map_pub.publish(output);
	*/
}

/**/
bool Uav_trajplannimg::res_for_des(auto_flight::destinate::Request& req,auto_flight::destinate::Response& res)
{
	ROS_INFO("get destination successfully");
	get_des_=1;
	des_coordinate.pose.position.x=req.x;
	des_coordinate.pose.position.y=req.y;
	des_coordinate.pose.position.z=req.z;
	res.des_r=1;
	return true;
}


uavTrans Uav_trajplannimg::get_trans_msg() const
{
	return  trans_msg;
}
geometry_msgs::PoseStamped Uav_trajplannimg::get_trans_msg_a() const
{
	return trans_msg_a;
}

geometry_msgs::PoseStamped Uav_trajplannimg::get_destinate_coor() const
{
	return des_coordinate;
}

int Uav_trajplannimg::get_get_des_() const
{
	return get_des_;
}

pcl::PointCloud<pcl::PointXYZ> Uav_trajplannimg::get_cloudmap() const
{
	return cloud;
}


void Uav_trajplannimg::pub_waypoint(geometry_msgs::PoseStamped way_point__)
{
	double rpy_[3];
	double quat_[4];
	rpy_[0]=0.0;
	rpy_[1]=0.0;
	rpy_[2]=atan2(trans_msg_a.pose.position.y-trans_msg_a1.pose.position.y,trans_msg_a.pose.position.x-trans_msg_a1.pose.position.x);
	bot_roll_pitch_yaw_to_quat (rpy_, quat_);
	way_point_.pose.position.x=way_point__.pose.position.x;
	way_point_.pose.position.y=way_point__.pose.position.y;
	way_point_.pose.position.z=way_point__.pose.position.z;
	
	if (rpy_[2]*180/3.1415!=0)  //偏航角变化小于一定的值才才重新赋值，保持偏航角稳定性
	{
      way_point_.pose.orientation.w=quat_[0];
	  way_point_.pose.orientation.x=quat_[1];
	  way_point_.pose.orientation.y=quat_[2];
	  way_point_.pose.orientation.z=quat_[3];
	  fly_yaw=rpy_[2]*180/3.1415;
	}
	waypoint_pub.publish(way_point_);
	way_point_path.poses.push_back(way_point_);
	way_point_path.header.frame_id="map";
	plan_way_pub.publish(way_point_path);
}

bool Uav_trajplannimg::arrive_destination(geometry_msgs::PoseStamped trans_msg_a,geometry_msgs::PoseStamped des_coordinate) const
{
	double x_distance=abs(trans_msg_a.pose.position.x-des_coordinate.pose.position.x);
	double y_distance=abs(trans_msg_a.pose.position.y-des_coordinate.pose.position.y);
	double z_distance=abs(trans_msg_a.pose.position.z-des_coordinate.pose.position.z);
	if (x_distance<2.0&&y_distance<2.0&&z_distance<2.0)
	{
		return true;
	}else
	return false;
}

void Uav_trajplannimg::fly_init()
{
	 while (get_des_!=1)
   {
	   ros::spinOnce();
	   des_coordinate_=des_coordinate;
   }
   while (TrajLibrarymain.LoadLibrary("/home/xxz/auto_flight/xxz_auto_flight/src/auto_flight/trajlib", 0)!=1)
   {
	    ros::spinOnce();
   }
   uavTrans start_p=set_trans_init();
   std::cout<<"setting destination straight trajectory"<<std::endl;
  if(TrajLibrarymain.set_desstraight_traj(des_coordinate_,start_p)!=true)
  {
	  std::cout<<"failed to set destination straight trajectory"<<std::endl;
  }
  TrajLibrarymain.Print();

  while(ros::ok() &&!current_state.connected)  //感觉这个地方有问题，应该是没连接，等待，
    {   
        ROS_INFO("The current state of connect is %d disconnect",current_state.connected);                                                
        ros::spinOnce();                            
        //rate.sleep();
    }
   ROS_INFO("The current state of connect is %d disconnect",current_state.connected);    
    //设置一个初始点位姿　（００２）点　悬停
    geometry_msgs::PoseStamped way_point_local;
    way_point_local.pose.position.x = 0;
    way_point_local.pose.position.y = 0;
    way_point_local.pose.position.z = 1.4;

	//publish some topic 
    for(int i=0; ros::ok()&&i<80;i++)
    {
		delay_msec(50.0);
        ROS_INFO("ros set begin point");
        waypoint_pub.publish(trans_msg_a); 
        ros::spinOnce(); //在这个过程中也授控制权，以便订阅话题和服务
    }

	mavros_msgs::SetMode offb_set_mode; //这是一个服务的消息类型　
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd; //这个是一个服务的消息类型
    arm_cmd.request.value = true; //true是解锁
    
    ros::Time last_request = ros::Time::now();

      //若当前不是　offboard模式　
	if(current_state.mode != "OFFBOARD" )
    {
      //  (ros::Time::now()-last_request>ros::Duration(3.0));
        ROS_INFO_STREAM("The current state of mode is  "<<current_state.mode);  //打印mode状态
       // delay(2);
	  if(set_mode_client.call(offb_set_mode)&&offb_set_mode.response.mode_sent)//发布offboard服务请求，并等待回复成功
	    ROS_INFO("Offboard enabled");

 	  last_request=ros::Time::now();//获取当前时间　　上面有一个作差，控制service-client 请求频率　３s/次
    }
    
    //若当前是为解锁状态   这中间有一个问题，
	  if(!current_state.armed)
      {
          ROS_INFO("The current state of arming is %d  disarmed",current_state.armed);//发布解锁服务请求，并等待回复成功
  	     // delay(2);
        if(arming_client.call(arm_cmd)&&arm_cmd.response.success)
		  ROS_INFO("Vehicle armed   success");		
		
  	      last_request=ros::Time::now();
	}
}


void Uav_trajplannimg::fly_takeoff(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* octree_map_)
{
	std::tuple<int, double, Trajectory*,std::vector<geometry_msgs::PoseStamped>> takeoff_traj{ TrajLibrarymain.FindFarthestTrajectory(safe_dis,trans_msg1,octree_map_,take_off_traj) };
	farthest_traj = std::get<2>(takeoff_traj); 
	way_point.assign(std::get<3>(takeoff_traj).begin(),std::get<3>(takeoff_traj).end()); 
	this_tra = TrajLibrarymain.GetThisTrajNum();
	do
	{
		ros::spinOnce();
		trans_msg_a1=trans_msg_a;
		std::cout<<"飞行高度:"<<trans_msg_a1.pose.position.z<<"米"<<std::endl;
		std::cout << "选择轨迹：" << this_tra << endl;	
		if (way_point_i<farthest_traj->GetNumberOfPoints()-1)
		{
			way_point_i++;
		}
		pub_waypoint(way_point.at(farthest_traj->GetNumberOfPoints()-1));
		delay_msec(10.0);
	}while(trans_msg_a1.pose.position.z<=4.1);
		//此处要调用回调函数中订阅到的位姿话题和地图话题，进行初始话的选择路径以及检测飞行高度，在达到设定导读之前需要
		//设置一个循环不断的调用 ros::spinOnce();语句来得到位姿，防止程序向下运行
	
	trans_msg1=trans_msg;
	trans_msg_a1=trans_msg_a;
	if (TrajLibrarymain.set_desstraight_traj(des_coordinate_,trans_msg1)!=true)
	{
	   std::cout<<"failed to set destination straight trajectory"<<std::endl;
	 }		

	 chosetraj=TrajLibrarymain.FindFarthestTrajectory(safe_dis,trans_msg1,octree_map_,straight_pre_traj_no); //初始飞行 朝向终点飞行
	this_tra = TrajLibrarymain.GetThisTrajNum();       //路径编号（初始值一般为4）
	traj_closest_dist = std::get<1>(chosetraj);        //路劲离最近障碍物的距离
	farthest_traj = std::get<2>(chosetraj);            //得到路径对象（位置坐标，姿态角） 将这个信息传给飞控
    way_point.clear();
	way_point.assign(std::get<3>(chosetraj).begin(),std::get<3>(chosetraj).end()); 

	way_point_i=0; 
	start = clock();  
}

bool Uav_trajplannimg::fly_traj_plan(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* octree_map_)
{
	
	if (way_point_i<farthest_traj->GetNumberOfPoints()-1)
	{
		way_point_i++;
	}
	std::cout<<"way_point_i:\n"<<way_point_i<<std::endl;
	//std::cout<<"way_point:\n"<<way_point.at(way_point_i)<<"\n"<<std::endl;
	pub_waypoint(way_point.at(8));
	std::cout<<"偏航角yaw="<<fly_yaw<<"度"<<std::endl;
	delay_msec(10.0); //在发布航点与重新做轨迹选择中间加入延时
	ros::spinOnce();

    finish = clock();
	totaltime = (float)(finish - start) / CLOCKS_PER_SEC;
	cout << "轨迹库运行时间：" << totaltime << "秒！" << endl;
	std::cout<<"当前位置:\n"<< trans_msg1.uav_position[0]<<"\n"<<trans_msg1.uav_position[1]<<"\n"<<trans_msg1.uav_position[2]<<std::endl;
				
	current_position[0]=trans_msg1.uav_position[0];
	current_position[1]=trans_msg1.uav_position[1];
	current_position[2]=trans_msg1.uav_position[2];
	current_point_distance=NearestNeighbor(current_position,octree_map_);
	if (closest_current_point_distance>current_point_distance||closest_current_point_distance==-1.0)
	{
		closest_current_point_distance=current_point_distance;
	}				

	std::cout << "当前位置点离障碍物最近距离=" << current_point_distance<< "米" << std::endl;
	std::cout << "历史当前位置点离障碍物最近距离=" << closest_current_point_distance<< "米\n" << std::endl;
				
	if (current_point_distance<0.3&&current_point_distance>0)
	{
		std::cout<<"飞机飞行状态危险"<<std::endl;
		while (ros::ok()&&current_state.mode == "OFFBOARD" )
		{  
			ros::spinOnce();
			pub_waypoint(trans_msg_a);
		}
	    while (ros::ok()&&current_state.mode != "OFFBOARD" )
		{
			ros::spinOnce();
		}		
		fly_init();
		//切换成手动，不要让程序终止,手动调整之后能切回来,要重新初始化连接
	}
	//TrajLibrarymain.Print();

	point_distances = farthest_traj->ClosestObstacleInRemainderOfTrajectory(totaltime, trans_msg1,octree_map_,0);  //想一想这个时间怎么去计时利用time函数？优化  检测最后一点
	if (closest_point_distances>point_distances||closest_point_distances==-1.0)
	{
		closest_point_distances=point_distances;
	}				
	std::cout << "当前运行轨迹离障碍物离最近距离=" <<point_distances << "米" << std::endl;
	cout << "当前轨迹：" << this_tra << endl;
	//cout << "当前选择轨迹离障碍最近物距离：" << traj_closest_dist <<"米"<< endl;
	cout << "此次轨迹库距离搜索时间：" << thisseartime << "秒！" << endl;
	std::cout << "历史选择轨迹离障碍物最近距离=" << closest_point_distances << "米" << std::endl;
							
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//trans_msg_a1=trans_msg_a;

			   //有障碍物
				if (point_distances<safe_dis&&point_distances>0)
				{  
					if (this_tra == 0 || this_tra == 4 || this_tra == 3) //悬停部分可以放在这里
					{
						if (this_tra_last = 1)
						{
					     	ros::spinOnce();
				            trans_msg1=trans_msg;
   						    trans_msg_a1=trans_msg_a;
						    if (TrajLibrarymain.set_desstraight_traj(des_coordinate_,trans_msg1)!=true)
						    {
							    std::cout<<"failed to set destination straight trajectory"<<std::endl;
						    }							     
							chosetraj= TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg1,octree_map_,straightl_pre_traj);
							//上次左飞，这次直线飞行后优先右飞
						}
						else
						{
							ros::spinOnce();
				            trans_msg1=trans_msg;
   						    trans_msg_a1=trans_msg_a;
						    if (TrajLibrarymain.set_desstraight_traj(des_coordinate_,trans_msg1)!=true)
						    {
							    std::cout<<"failed to set destination straight trajectory"<<std::endl;
						    }	
						    chosetraj= TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg1,octree_map_,straightr_pre_traj);
							//上次右飞，这次直线飞行后优先左飞
						}
					}
					else if (this_tra == 1)
					{
						ros::spinOnce();
				        trans_msg1=trans_msg;
   						trans_msg_a1=trans_msg_a;
					    if (TrajLibrarymain.set_desstraight_traj(des_coordinate_,trans_msg1)!=true)
						{
						    std::cout<<"failed to set destination straight trajectory"<<std::endl;
						}	
						chosetraj=TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg1,octree_map_,left_pre_traj);
					}
					else 
					{
						ros::spinOnce();
				         trans_msg1=trans_msg;
   						trans_msg_a1=trans_msg_a;
						if (TrajLibrarymain.set_desstraight_traj(des_coordinate_,trans_msg1)!=true)
						{
							std::cout<<"failed to set destination straight trajectory"<<std::endl;
						 }	
						chosetraj= TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg1,octree_map_, right_pre_traj);
					}
					this_tra_last = this_tra;//上次的路劲编号
					this_tra = TrajLibrarymain.GetThisTrajNum();
					traj_closest_dist = std::get<1>(chosetraj);    //路劲离最近障碍物的距离
					farthest_traj = std::get<2>(chosetraj);        //得到路径对象（位置坐标，姿态角） 将这个信息传给飞控，利用ROS消息的形式
                    way_point.clear();
				     way_point.assign(std::get<3>(chosetraj).begin(),std::get<3>(chosetraj).end()); 
															  //在获取路径之后记录一下此时的无人机位置以及姿态角，
															  //并将轨迹中的数据全部转化成地面坐标系的数据存入到x_ground_point数组当中
					t_last = totaltime;
					finish = clock();
					totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
					thisseartime = totaltime - t_last;

					cout << "重新选择轨迹：" << this_tra << endl;
					//cout << "此次轨迹库距离搜索时间：" << thisseartime << "秒！" << endl;

					start = clock();
					way_point_i=0;
				}

				//一直无障碍
				//比较当前时刻和轨迹库中的时刻的最大值（或者是此时无人机的位置与之前转化的x_ground_point中的差值小于设定误差值），
				//若大于，则选择无障碍物的时的选择
				else if (way_point_i==8) //(farthest_traj->GetNumberOfPoints()-1)  && point_distances>safe_dis
				{  
					if (this_tra==1)
					{
						ros::spinOnce();
				         trans_msg1=trans_msg;
   						 trans_msg_a1=trans_msg_a;
						if (TrajLibrarymain.set_desstraight_traj(des_coordinate_,trans_msg1)!=true)
						{
							std::cout<<"failed to set destination straight trajectory"<<std::endl;
						}
						chosetraj= TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg1,octree_map_,left_pre_traj_no);
					}
					else if (this_tra==2)
					{
						ros::spinOnce();
				         trans_msg1=trans_msg;
   						 trans_msg_a1=trans_msg_a;
						if (TrajLibrarymain.set_desstraight_traj(des_coordinate_,trans_msg1)!=true)
						{
							std::cout<<"failed to set destination straight trajectory"<<std::endl;
						}
						chosetraj= TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg1,octree_map_, right_pre_traj_no);
					}
					else
					{
						ros::spinOnce();
				         trans_msg1=trans_msg;
   						 trans_msg_a1=trans_msg_a;
						if (TrajLibrarymain.set_desstraight_traj(des_coordinate_,trans_msg1)!=true)
						{
							std::cout<<"failed to set destination straight trajectory"<<std::endl;
							return 0;
						}
						chosetraj =TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg1,octree_map_,straight_pre_traj_no) ;
					//return 0;
					}

					this_tra_last = this_tra;//上次的路劲编号
					this_tra = TrajLibrarymain.GetThisTrajNum();
					traj_closest_dist = std::get<1>(chosetraj);    //路劲离最近障碍物的距离
					farthest_traj = std::get<2>(chosetraj);        //得到路径对象（位置坐标，姿态角） 将这个信息传给飞控，利用ROS消息的形式
                    way_point.clear();
					way_point.assign(std::get<3>(chosetraj).begin(),std::get<3>(chosetraj).end()); 
                    
				    t_last = totaltime;
					finish = clock();
					totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
				    thisseartime = totaltime - t_last;
					//cout << "此次轨迹库距离搜索时间：" << thisseartime << "秒！" << endl;
					cout << "重新选择轨迹：" << this_tra << endl;

					start = clock();
					way_point_i=0;
				}
   return true;
}