#include "ros/ros.h"                                    // ROS 기본 헤더파일
#include "cmd_vel_topic/msgCmdVel.h"            // msgCmdVel 메시지 파일 헤더 (빌드후 자동 생성됨)

int main(int argc, char **argv)                         // 노드 메인 함수
{
  ros::init(argc, argv, "cmd_vel_topic");  // 노드명 초기화
  ros::NodeHandle nh;                                   // ROS 시스템과 통신을 위한 노드 핸들 선언

  // 발행자 선언, cmd_vel_topic 패키지의 msgCmdVel 메시지 파일을 이용한
  // 발행자 cmd_vel_pub 를 작성한다. 토픽명은 "cmd_vel" 이며,
  // 발행자 큐(queue) 사이즈를 100개로 설정한다는 것이다
  ros::Publisher ros_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

  // 루프 주기를 설정한다. "10" 이라는 것은 10Hz를 말하는 것으로 0.1초 간격으로 반복된다
  ros::Rate loop_rate(1);

  int count = 0;    // 메시지에 사용될 변수 선언

//  cmd_vel_topic::msgCmdVel msg;      // msgCmdVel 메시지 파일 형식으로 msg 라는 메시지를 선언
  geometry_msgs::Twist msg;
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;
  while (ros::ok())
  {
//    msg.data = count;                   // count 라는 변수를 이용하여 메시지 값을 정한다

	std::cout << "input velocity" << std::endl;
	std::cin >> msg.linear.x >> msg.angular.z ;
/*
    ROS_INFO("send msg = %d", count);   // ROS_INFO 라는 ROS 함수를 이용하여 count 변수를 표시한다

	msg.stamp = ros::Time::now();
    ROS_INFO("send sec = %d", msg.stamp.sec);   // ROS_INFO 라는 ROS 함수를 이용하여 count 변수를 표시한다
    ROS_INFO("send sec = %d", msg.stamp.nsec);   // ROS_INFO 라는 ROS 함수를 이용하여 count 변수를 표시한다

	msg.str = "ros_stamp_"+std::to_string(count);
    ROS_INFO("send str = %s", msg.str.c_str());   // ROS_INFO 라는 ROS 함수를 이용하여 count 변수를 표시한다

*/
    ros_cmd_vel_pub.publish(msg);      // 메시지를 발행한다. 약 0.1초 간격으로 발행된다

    loop_rate.sleep();                  // 위에서 정한 루프 주기에 따라 슬립에 들어간다

//    ++count;                            // count 변수 1씩 증가
  }

  return 0;
}
