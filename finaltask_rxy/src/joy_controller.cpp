#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h" 


class Joy2Cmd
{
public:
  Joy2Cmd() : _nh()
  {
    //ノードのSubscriberを設定
    //第一引数:Subscribe対象トピック, 第二引数:バッファーサイズ, 第三引数:メッセージを受け取った際のコールバック関数
    //subscribe対象のトピックが更新される度にメッセージを受け取り、コールバック関数が呼び出される
    _joy_sub = _nh.subscribe("/joy", 1, &Joy2Cmd::joyCallback, this);
    //ノードのPublisherを設定
    //第一引数:Publish先トピック, 第二引数:バッファーサイズ
    _joy_state_pub = _nh.advertise<std_msgs::Float32>("joy_state", 10, true);
    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true); 
  };
  ~Joy2Cmd(){};
  void run()
  {
    //メッセージがpublishされるたびにコールバック関数にアクセスする処理
    ros::spin();
  }

private:
  // ros
  ros::NodeHandle _nh;
  ros::Subscriber _joy_sub;
  ros::Publisher _joy_state_pub;
  ros::Publisher _cmd_vel_pub; 

  // callback関数の設定
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    std_msgs::Float32 joy_st;

    // 0左コントローラー左右
    // 1左コントローラー前後
    // 2右コントローラー左右
    // 3右コントローラー前後
    //コントローラーの入力をメッセージに格納
    joy_st.data = msg->axes[1];
    
    // add code for YB control by joycon
    geometry_msgs::Twist twist;
    twist.linear.x = msg->axes[1]*0.2;
    twist.angular.z = msg->axes[2]*0.2;
    _cmd_vel_pub.publish(twist);

    //対象トピック(joy_state)にPublish
    _joy_state_pub.publish(joy_st);
  }
};


int main(int argc, char **argv)
{
  //ノードの初期化
  ros::init(argc, argv, "joy_contoroller_node");
  Joy2Cmd node;

  node.run();
  return 0;
}

