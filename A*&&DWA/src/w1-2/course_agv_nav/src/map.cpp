#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;


ros::Publisher map_pub;
ros::Subscriber obs_x_sub;
ros::Subscriber obs_y_sub;

vector<vector<float>> maps;
vector<vector<float>> obs_list(2);

int x_min, x_max;
int y_min, y_max;
int width;
int height;
float grid_size = 0.2;
float robot_radius = 0.6;

int obs_rcv_flag = 0;


void obs_x_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    unsigned int msg_size = msg->data.size();
    obs_list[0] = {};
    for(int i = 0; i < msg_size; i++)
    {
        obs_list[0].push_back(msg->data.at(i));
    }
    obs_rcv_flag++;
    ROS_INFO("obs_x_receved:%d",obs_list[0].size());
}

void obs_y_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    unsigned int msg_size = msg->data.size();
    obs_list[1] = {};
    for(int i = 0; i < msg_size; i++)
    {
        obs_list[1].push_back(msg->data.at(i));
    }
    obs_rcv_flag++;
    ROS_INFO("obs_y_receved:%d",obs_list[1].size());
}

float getMax(vector<float> list)
{
    float max_val = -999999999999999;
    for(int i=0; i < list.size(); i++)
    {
        max_val = (max_val < list[i])?list[i]:max_val;
    }
    return max_val;
}

float getMin(vector<float> list)
{
    float min_val = 99999999999999;
    for(int i=0; i < list.size(); i++)
    {
	min_val = (min_val > list[i])?list[i]:min_val;
    }
    return min_val;
}

float idx2coor(int idx, int min)
{
    return (idx + 0.5) * grid_size + min;
}

void genMap()
{
    ROS_INFO("BUILDING MAP");
    x_min = (int)round(getMin(obs_list[0]));
    x_max = (int)round(getMax(obs_list[0]));
    y_min = (int)round(getMin(obs_list[1]));
    y_max = (int)round(getMax(obs_list[1]));

    width = (int)((round(x_max) - round(x_min)) / grid_size)+1;
    height = (int)((round(y_max) - round(y_min)) / grid_size)+1;

    maps.resize(width);
    for(int ix = 0; ix < width; ix++)
    {
        maps[ix].resize(height);
    }

    for(int i = 0; i < obs_list[0].size(); i++)
    {
        float obs_xmin = obs_list[0][i] - robot_radius;
        float obs_ymin = obs_list[1][i] - robot_radius;
        float obs_xmax = obs_list[0][i] + robot_radius;
        float obs_ymax = obs_list[1][i] + robot_radius;
        
        float obs_xmin_idx = int((obs_xmin - x_min) / grid_size);
        float obs_ymin_idx = int((obs_ymin - y_min) / grid_size);
        float obs_xmax_idx = int((obs_xmax - x_min) / grid_size);
        float obs_ymax_idx = int((obs_ymax  - y_min) / grid_size);
        //cout<<"xmin_idx"<<xmin_idx<<"ymin_idfx"<<ymin_idx<<endl;
        for(int j = (obs_xmin_idx >= 0?obs_xmin_idx:0); j <= (obs_xmax_idx < width-1?obs_xmax_idx:width-1); j++)
        {
            //cout<<"j"<<j<<endl;
            for(int k = (obs_ymin_idx >=0 ?obs_ymin_idx:0); k <= (obs_ymax_idx < height-1?obs_ymax_idx:height-1); k++)
            {
                //cout<<"k"<<k<<endl;
                maps[j][k] = 1;
            }
        }
    }
    ROS_INFO("FINISHED BUILDING MAP, WIDTH:  %d, HEIGHT: %d", width, height);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_server");
    ros::NodeHandle nh;

    ros::Rate rate(20);

    map_pub = nh.advertise<std_msgs::Float64MultiArray>("/maps",10);

    obs_x_sub = nh.subscribe("/obs_x", 10, obs_x_callback);
    obs_y_sub = nh.subscribe("/obs_y", 10, obs_y_callback);
  
    while(ros::ok())
    {
        if(obs_rcv_flag == 2)
        {
           ros::Time start = ros::Time::now();
            genMap();
            cout<<"generate map cost:"<<ros::Time::now() - start<<endl;
            
            obs_rcv_flag = false;
            std_msgs::Float64MultiArray msg;
            
            for(int i=0; i<maps.size();i++)
            {
                for(int j = 0;j<maps[i].size();j++)
                {
                    /*
                        if(maps[i][j]==1)
                            cout<<"#";
                        else
                            cout<<" ";
                    */
                    msg.data.push_back(maps[i][j]);
                }
                //cout<<endl;
            }
            
            map_pub.publish(msg);            
        }
        
        ros::spinOnce();
        rate.sleep();
    }
}
