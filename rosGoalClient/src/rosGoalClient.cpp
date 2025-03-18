#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <signal.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "rosGoalClient/bot3gpio.h"

#define NODE_NAME	"rosGoalClient"
#define SRV_NAME	"gpio_server"

#define BUF_SIZE	100
#define NAME_SIZE	20
#define ARR_CNT		10
#define GOALCNT		2

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void * send_msg(void * arg);
void * recv_msg(void * arg);
void error_handling(const char *msg);

char name[NAME_SIZE] = "[Default]";
char msg[BUF_SIZE];


int main(int argc, char *argv[])
{
	int sock;
	struct sockaddr_in serv_addr;
	pthread_t snd_thread, rcv_thread;
	void * thread_return;

	ros::init(argc, argv, NODE_NAME);
	if(argc != 4) {
		printf("Usage :rosrun rosGoalClient %s <IP> <port> <name>\n", argv[0]);
		exit(1);
	}

	
	sprintf(name, "%s", argv[3]);
	
	sock = socket(PF_INET, SOCK_STREAM, 0);
	if(sock == -1)
		error_handling("socket() error");

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family=AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
	serv_addr.sin_port = htons(atoi(argv[2]));

	if(connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
	error_handling("connect() error");
	sprintf(msg, "[%s:PASSWD]", name);
	write(sock, msg, strlen(msg));
	pthread_create(&rcv_thread, NULL, recv_msg, (void *)&sock);
	pthread_create(&snd_thread, NULL, send_msg, (void *)&sock);

	pthread_join(snd_thread, &thread_return);
	pthread_join(rcv_thread, &thread_return);

	close(sock);
	return 0;
}

void *send_msg(void *arg)
{
	int *sock = (int *)arg;
	int str_len;
	int ret;
	fd_set initset, newset;
	struct timeval tv;
	char name_msg[NAME_SIZE + BUF_SIZE+2];

	FD_ZERO(&initset);
	FD_SET(STDIN_FILENO, &initset);

	fputs("Input a message! [ID]msg (Default ID:ALLMSG)\n",stdout);
	while(1) {
		memset(msg,0,sizeof(msg));
		name_msg[0]='\0';
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		newset = initset;
		ret = select(STDIN_FILENO + 1, &newset, NULL, NULL, &tv);
		if(FD_ISSET(STDIN_FILENO, &newset))
		{
			fgets(msg, BUF_SIZE, stdin);
			if(!strncmp(msg,"quit\n",5)) 
			{
				*sock = -1;
				return NULL;
			}
			else if(msg[0] != '[')
			{
				strcat(name_msg, "[ALLMSG]");
				strcat(name_msg, msg);
			}
			else
				strcpy(name_msg,msg);
			if(write(*sock, name_msg, strlen(name_msg))<=0)
			{
				*sock = -1;
				return NULL;
			}
		}
		if(ret == 0)
		{
			if(*sock == -1)
				return NULL;
		}
	}
}

void *recv_msg(void *arg)
{
	double dSeqVal[GOALCNT][3] = {
			{2.69, 0.07, 0.99},	\
			{0.0, 0.0, 0.86},	\
	};

	int *sock = (int *)arg;
	int i;
	char *pToken;
	char *pArray[ARR_CNT]={0};

	char name_msg[NAME_SIZE + BUF_SIZE + 1];
	int str_len;


	ros::NodeHandle nh;

	ros::ServiceClient srv_client;
	srv_client = nh.serviceClient<rosGoalClient::bot3gpio>(SRV_NAME);
	rosGoalClient::bot3gpio srv;
	srv.request.a = 0;
	srv.request.b = 1;

	MoveBaseClient ac("move_base", true);
	move_base_msgs::MoveBaseGoal goal;

	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	goal.target_pose.header.frame_id = "map";

	while(1) {
		memset(name_msg, 0x0, sizeof(name_msg));
		str_len = read(*sock, name_msg, NAME_SIZE + BUF_SIZE);
		if(str_len<= 0)
		{
			*sock = -1;
			return NULL;
		}
		name_msg[str_len-1] = 0;
		fputs(name_msg, stdout);

		pToken = strtok(name_msg, "[:@]");
		i = 0;
		while(pToken != NULL)
		{
			pArray[i] = pToken;
			if(i++ >= ARR_CNT)
					break;
			pToken = strtok(NULL, "[:@]");
		}
		if(!strcmp(pArray[1], "GPIO")){
			if(!strcmp(pArray[2], "ON"))
				srv.request.a = 0;
			else
				srv.request.a = 1;
			sprintf(name_msg, "[%s]%s%s\n",pArray[0],pArray[1],pArray[2]);
			write(*sock, name_msg, strlen(name_msg));
			if(srv_client.call(srv))
			{
				ROS_INFO("send srv, srv.result : %ld", srv.request.a);
				ROS_INFO("recieve srv, srv.Response.result: %ld", srv.response.result);
			}
			else
			{
				ROS_ERROR("Failed to call service");
			}
		}
		else if(!strcmp(pArray[1], "GOGOAL") && (i==5)){
			printf("GOGOAL");
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position.x = strtod(pArray[2], NULL);
			goal.target_pose.pose.position.y = strtod(pArray[3], NULL);
			goal.target_pose.pose.orientation.w = strtod(pArray[4], NULL);
			ROS_INFO("Sending goal 1");
			ac.sendGoalAndWait(goal, ros::Duration(10.0,0), ros::Duration(10.0,0));

			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				ROS_INFO("Goal arrived!");
			else
				ROS_INFO("The base failed to move to goal for some reason");
		}
		else if(!strcmp(pArray[1], "TURTLE")){
			printf("TURTLE");
			sprintf(name_msg, "[ALLMSG]TURTLEGO\n");
			write(*sock, name_msg, strlen(name_msg));
			for (int j=0; j<GOALCNT; j++){
				goal.target_pose.header.stamp = ros::Time::now();

				goal.target_pose.pose.position.x = dSeqVal[j][0];
				goal.target_pose.pose.position.y = dSeqVal[j][1];
				goal.target_pose.pose.orientation.w = dSeqVal[j][2];
				ROS_INFO("Sending goal Seq1");
				ac.sendGoalAndWait(goal, ros::Duration(20.0,0), ros::Duration(20.0,0));

				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					ROS_INFO("Goal Seq1!");
				else
					ROS_INFO("The base failed to move to goal for some season");
			}
			sprintf(name_msg, "[ALLMSG]TURTLESTOP\n");
			write(*sock, name_msg, strlen(name_msg));
		}
		printf("id:%s, msg:%s\n",pArray[0], pArray[1]);
	}
}

void error_handling(const char *msg)
{
	fputs(msg, stderr);
	fputc('\n', stderr);
	exit(1);
}
