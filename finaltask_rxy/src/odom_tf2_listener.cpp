#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <ros/time.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// #include <turtlesim/Spawn.h>
// 列挙体で状態名を定義しておくことで可読性を向上
enum State
{
    STOP,
    RUN,
    OBSTACLE,
};

int judge_state = STOP;
bool obs_finish_flag = true;  //時刻を取得したか判定するフラグ
bool point_wait_flag = true; //within 2s = true
bool close_goal = false;
bool time_count_falg = false;
ros::Time found_obs_time = ros::Time();
ros::Duration continuous_found_duration;

ros::Time goal_time;
geometry_msgs::Twist vel_msg; // 指令する速度、角速度
// オドメトリから得られる現在の位置と姿勢
double robot_x, robot_y;
geometry_msgs::Quaternion robot_r;
double roll, pitch, yaw;
geometry_msgs::PoseStamped obs_goal; // 障害物回避用
sensor_msgs::LaserScan scan;
ros::Time get_point_time = ros::Time(0); //initialize

// // オドメトリのコールバック
// void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
// {
// 	robot_x = msg->pose.pose.position.x;
// 	robot_y = msg->pose.pose.position.y;
// 	robot_r = msg->pose.pose.orientation;
// }

// クォータニオンをオイラーに変換                                               
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_)
{
    // sensor_msgs::LaserScan scan;
    scan.header = scan_->header;
    scan.angle_min = scan_->angle_min;
    scan.angle_max = scan_->angle_max;
    scan.angle_increment = scan_->angle_increment;
    scan.time_increment = scan_->time_increment;
    scan.scan_time = scan_->scan_time;
    scan.range_min = scan_->range_min;
    scan.range_max = scan_->range_max;
    scan.ranges = scan_->ranges;
    scan.intensities = scan_->intensities;

    //障害物検出 
    double min_distance = std::numeric_limits<double>::infinity();
    double max_distance = 0;
	double angle_of_closest;
    double angle_of_farest;
    bool found = false;
    bool time_wait_flag = true;

	for (int i = (scan.ranges.size())/3; i < (scan.ranges.size())*2/3; ++i) {
		double distance = scan.ranges[i];
		// std::cout << "dis1 now = " << distance1 << std::endl;
		if (distance >= 0.3 && distance < 0.5) {
			if (distance < min_distance) {
				min_distance = distance;
				angle_of_closest = scan.angle_min + i * scan.angle_increment;
				// std::cout << "P1:(dis,angle):" << min_distance << ", " << angle_of_closest << std::endl;
				found = true;
			}
            //URG sensors are rotate counterclockwise, so get right part max_distance
            // if(i < (scan.ranges.size())/2){
                if (distance > max_distance) {
                    max_distance = distance;
                    angle_of_farest = scan.angle_min + i * scan.angle_increment;
                }
            // }
		}
	}

    if(obs_finish_flag){//障害物回避完了まで状態変化しない
    // ROS_INFO("SSSSCCCAAANNN: get into if");
        if (found){
            // ROS_INFO("FOUND!!!!!!");
            std::cout << "Obs min:(dis,angle):" << min_distance << ", " << angle_of_closest << std::endl;   
            if (!time_count_falg) {
                // ROS_INFO("GET into SET TIME!!!!!");
                time_count_falg = true;
                found_obs_time = ros::Time::now();
                continuous_found_duration = ros::Duration(0);
            } else {
                // ROS_INFO("GET into COUNT!!!!!");
                time_count_falg = true;
                continuous_found_duration = ros::Time::now() - found_obs_time;
                std::cout << "WAIT COUNT:" << continuous_found_duration.toSec() << std::endl;  
                if (continuous_found_duration.toSec() >= 3.0) {
                    time_wait_flag = false;
                }
            }
            // std::cout << "time_wait_flag 2:" << time_wait_flag << std::endl;  
            if (time_wait_flag) {
                // wait inside 3s set to stop state
                judge_state = STOP;
            }else{
                judge_state = OBSTACLE;
            }
            
            // 障害物あり
            //現在値利用し、回避用ゴール計算
            double obstacle1_x = min_distance * cos(angle_of_closest);
            double obstacle1_y = min_distance * sin(angle_of_closest);
            double obstaclefar_x = max_distance * cos(angle_of_farest);
            double obstaclefar_y = max_distance * sin(angle_of_farest);
            double world_obstacle1_x = 0;
            double world_obstacle1_y = 0;
            double world_obstaclefar_x = 0;
            double world_obstaclefar_y = 0;
            //obstacle in world 
            world_obstacle1_x = robot_x + obstacle1_x*cos(yaw)-obstacle1_y*sin(yaw);
            world_obstacle1_y = robot_y + obstacle1_x*sin(yaw)+obstacle1_y*cos(yaw);
            world_obstaclefar_x = robot_x + obstaclefar_x*cos(yaw)-obstaclefar_y*sin(yaw);
            world_obstaclefar_y = robot_y + obstaclefar_x*sin(yaw)+obstaclefar_y*cos(yaw);
            //set goal 
            std::cout << "wrold obs x min, x max:" << world_obstacle1_x << ", " << world_obstaclefar_x << std::endl;
            std::cout << "Robot y:" << robot_y << std::endl;
            std::cout << "wrold obs y min, y max:" << world_obstacle1_y << ", " << world_obstaclefar_y << std::endl;
            // need think more here!!!!!!!!!!!!!!!!!!!
            obs_goal.pose.position.x = (world_obstacle1_x + world_obstaclefar_x)/2;
            if(robot_y > world_obstaclefar_y){
                ROS_INFO("TURN LEFT");
                obs_goal.pose.position.y = world_obstacle1_y + 0.5;
            }else{
                ROS_INFO("TURN RIGHT");
                obs_goal.pose.position.y = world_obstacle1_y - 0.5;
            }
            
            std::cout << "OBS_GOAL robo x,y:" << obs_goal.pose.position.x  << ", " << obs_goal.pose.position.y << std::endl;
        }else{

            time_count_falg = false;
            // 障害物なし、通常走行
            judge_state = RUN;
        }
    }
}


void stop()
{
    ROS_INFO("NOW: STOP");
    vel_msg.angular.z = 0;  //角速度
    vel_msg.linear.x = 0;  //並進速度
    
}

void run(geometry_msgs::TransformStamped transformStamped, geometry_msgs::Transform geo_tf)
{
    ROS_INFO("NOW: RUN");
    ros::Rate rate(10.0);
    if (!close_goal){  //ゴールへ走行

        vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                        transformStamped.transform.translation.x);  //角速度

        vel_msg.linear.x = 0.3;  //安定速度で走行

        std::cout << "lisener goal:(x,y):" << transformStamped.transform.translation.x << ", " << transformStamped.transform.translation.y << std::endl;

        double dist = hypot(transformStamped.transform.translation.x,transformStamped.transform.translation.y);  //goalとrobotの距離計算
        std::cout << "lisener dist:" << dist << std::endl;
        if(dist < 0.015 && point_wait_flag){  //1.5cmずれまで許容
            close_goal = true;
        }

    }else{  //ゴール到着、geometryメッセージをtfに変換.姿勢をPublish

        tf2::Transform tf;
        tf2::convert(geo_tf,tf);
        double yaw_wp = tf2::getYaw(tf.getRotation());

        std::cout << "lisener yaw:" << yaw_wp << std::endl;
        if(abs(yaw_wp) < 5 * M_PI / 180.0){
            vel_msg.angular.z = 0;
            vel_msg.linear.x = 0;
            ros::param::set("/shared_flag", true);
            close_goal = false;
            //record current time
            get_point_time = ros::Time::now();
        }else{
            if (yaw_wp > 0) {
                // Turn clockwise
                vel_msg.angular.z = 0.3; // Negative angular velocity for clockwise rotation
            } else {
                // Turn counterclockwise
                vel_msg.angular.z = -0.3; // Positive angular velocity for counterclockwise rotation
            }
            vel_msg.linear.x = 0;
        }
    }
    
}

int near_position(geometry_msgs::PoseStamped goal)
{//　goalで指定した位置に近いかの判定を行う
	double difx = robot_x - goal.pose.position.x;
	double dify = robot_y - goal.pose.position.y;
    // std::cout << "Gaol dist now:" << sqrt(difx * difx + dify * dify) << std::endl;
	return (sqrt(difx * difx + dify * dify) < 0.15);
}

void go_position(geometry_msgs::PoseStamped goal)
{
    double k_v = 0.1; // 速度の係数
    // double k_v = 1.1; // 速度の係数test
    double k_w = 1.6; // 角速度の係数
	
	// 指令する速度と角速度
	double v = 0.0;
	double w = 0.0;

	//　角速度の計算
	double theta = atan2(goal.pose.position.y - robot_y, goal.pose.position.x - robot_x);
	while (theta <= -M_PI || M_PI <= theta)
	{
		if (theta <= -M_PI)
			theta = theta + 2 * M_PI;
		else
			theta = theta - 2 * M_PI;
	}

	// 現在のロボットのroll, pitch, yawを計算
	geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

	while (yaw <= -M_PI || M_PI <= yaw)
	{
		if (yaw <= -M_PI)
			yaw = yaw + 2 * M_PI;
		else
			yaw = yaw - 2 * M_PI;
	}

	theta = theta - yaw; //thetaに目標とする点に向くために必要となる角度を格納

	while (theta <= -M_PI || M_PI <= theta)
	{
		if (theta <= -M_PI)
			theta = theta + 2 * M_PI;
		else
			theta = theta - 2 * M_PI;
	}

	w = k_w * theta*2;

	// 速度の計算(追従する点が自分より前か後ろかで計算を変更)
	if (theta <= M_PI / 2 && theta >= -M_PI / 2)
		v = k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));
	else
		v = -k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));
	
	// publishする値の格納
	// vel_msg.linear.x = v;
    vel_msg.linear.x = 0.3;
	vel_msg.linear.y = 0.0;
	vel_msg.linear.z = 0.0;
	vel_msg.angular.x = 0.0;
	vel_msg.angular.y = 0.0;
	vel_msg.angular.z = w;

}

void obstacle()
{
    ROS_INFO("NOW: OBSTACLE");
    obs_finish_flag=false;

    // std::cout << "GOAL NOW:" << obs_goal<< std::endl;
    go_position(obs_goal);
    if (near_position(obs_goal))
    {
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        //障害物回避complete
        obs_finish_flag=true;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_listener");
    ros::NodeHandle nh;
    ros::Publisher base_link =
    nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);  //Publisher(geometry_msgs::Twist型メッセージをcmd_velに流す)
    // ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odom_callback);// no need this topic
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::NodeHandle pnh("~");
    std::string base_link_frame;
    std::string goal_frame;
    std::string odom_frame;
    pnh.getParam("base_link_id", base_link_frame);
    pnh.getParam("goal_id", goal_frame);
    pnh.getParam("odom_id", odom_frame);
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped transformStamped_odom;
    geometry_msgs::Transform geo_tf;
    tf2_ros::Buffer tfBuffer;  //TFメッセージを受信しそのメッセージからtf変換情報を取り出して保存
    tf2_ros::Buffer tfBuffer_odom;  //TFメッセージを受信しそのメッセージからtf変換情報を取り出して保存
    tf2_ros::TransformListener tfListener(tfBuffer);  //tf変換情報からTF変換を取得する
    tf2_ros::TransformListener tfListener_odom(tfBuffer_odom);  //tf変換情報からTF変換を取得する

    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Rate rate(10.0);
    State state = STOP;
    bool shared_eof_flag=false;

    while (ros::ok())
    {   
        ros::param::get("/shared_flag", shared_eof_flag);
        if (shared_eof_flag) {
            // end of file and stop
            // state = STOP;
            judge_state = STOP;
        }
        //get robot coordination data
        // try {
        //     // Listen for a transformation between "base_link" and "odom"
        //     listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
            
        //     // Print the translation and rotation of the transformation
        //     ROS_INFO("Translation: (%.2f, %.2f, %.2f)", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        //     ROS_INFO("Rotation: (%.2f, %.2f, %.2f, %.2f)", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
        // }
        // catch (tf::TransformException& ex) {
        //     // ROS_ERROR("%s", ex.what());
        // }
        // robot_x = transform.getOrigin().x();
        // robot_y = transform.getOrigin().y();
        // robot_r.x = transform.getRotation().x();
        // robot_r.y = transform.getRotation().y();
        // robot_r.z = transform.getRotation().z();
        // robot_r.w = transform.getRotation().w();
        
        //use tf2
        try {
            // Listen for a transformation between "base_link" and "odom"
            transformStamped_odom = tfBuffer_odom.lookupTransform("odom", "base_link", ros::Time(0));
            
            // Print the translation and rotation of the transformation
            ROS_INFO("Translation: (%.2f, %.2f, %.2f)", transformStamped_odom.transform.translation.x,
                                                        transformStamped_odom.transform.translation.y,
                                                        transformStamped_odom.transform.translation.z);
            ROS_INFO("Rotation: (%.2f, %.2f, %.2f, %.2f)", transformStamped_odom.transform.rotation.x,
                                                        transformStamped_odom.transform.rotation.y,
                                                        transformStamped_odom.transform.rotation.z,
                                                        transformStamped_odom.transform.rotation.w);
        }
        catch (tf2::TransformException& ex) {
            // ROS_ERROR("%s", ex.what());
        }

        robot_x = transformStamped_odom.transform.translation.x;
        robot_y = transformStamped_odom.transform.translation.y;
        robot_r.x = transformStamped_odom.transform.rotation.x;
        robot_r.y = transformStamped_odom.transform.rotation.y;
        robot_r.z = transformStamped_odom.transform.rotation.z;
        robot_r.w = transformStamped_odom.transform.rotation.w;
        switch (state)
        {
        case STOP:
            // if文の条件がイベント
            if (judge_state == RUN)
            {
                ROS_INFO("STOP to RUN");
                state = RUN;
            }else if (judge_state == OBSTACLE)
            {
                ROS_INFO("STOP to OBSTACLE");
                state = OBSTACLE;
            }
            break;
        case RUN:
            if (judge_state == STOP){
                ROS_INFO("RUN to stop");
                state = STOP;
            }else if (judge_state == OBSTACLE){
                ROS_INFO("RUN to OBSTACLE");
                state = OBSTACLE;
            }
            break;
        case OBSTACLE:
            if (judge_state == RUN)
            {
                ROS_INFO("OBSTACLE to RUN");
                state = RUN;
            }else if (judge_state == STOP)
            {
                ROS_INFO("OBSTACLE to STOP");
                state = STOP;
            }
            break;
        default:
            ROS_ERROR("error"); // この部分は絶対に実行されないはず
        }

        // 各状態での処理
        switch (state)
        {
        case STOP:
            stop();
            break;
        case RUN:
             if (!close_goal){ 
                try{
                    transformStamped = tfBuffer.lookupTransform(base_link_frame, goal_frame, ros::Time(0));  //parent_frame,child_frame
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());  //例外処理
                    ros::Duration(1.0).sleep();  //１秒スリープ
                }
            }else{
                try{
                    geo_tf = (tfBuffer.lookupTransform(base_link_frame,goal_frame,ros::Time(0))).transform;
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    ros::Duration(1.0).sleep();
                }
            }
            run(transformStamped,geo_tf);
            break;
        case OBSTACLE:
            obstacle();
            break;
        default:
            ROS_ERROR("error"); // この部分は絶対に実行されないはず
        }

        base_link.publish(vel_msg);

        if ((ros::Time::now() - get_point_time).toSec() <= 1.5) {
            // finish current goal within 2s
            point_wait_flag = false;
        }else{
            // get goal after 2s
            point_wait_flag = true;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};

