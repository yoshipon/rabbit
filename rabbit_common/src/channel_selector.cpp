#include <iostream>
#include <fstream>
#include <string>
#include <complex>
#include <random>
#include <cmath>

#include <boost/format.hpp>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <rabbit_msgs/Audio.h>

using namespace Eigen;
using namespace std;

class ChannelSelectorNode
{
  ros::Subscriber audio_sub;
  ros::Publisher audio_pub;

  rabbit_msgs::Audio audio_msg;

  vector<int> list_ch;
  
public:
  ChannelSelectorNode(int argc, char *argv[])
  {
    ros::init(argc, argv, "channel_selector");
    ros::NodeHandle n;

    ros::param::param<vector<int>>("~ch_list", list_ch, {0});
    std::cout << Map<MatrixXi>(list_ch.data(), 1, list_ch.size()) << std::endl;


    audio_pub = n.advertise<rabbit_msgs::Audio>("audio_out", 100);
    audio_sub = n.subscribe("audio_in", 100, &ChannelSelectorNode::audio_callback, this);

    audio_msg.nch = list_ch.size();
  }

  void audio_callback(const rabbit_msgs::Audio::ConstPtr &msg)
  {
    Map<const ArrayXXf> audio_in((float*)msg->data.data(), msg->nch, msg->len);

    audio_msg.len = msg->len;
    audio_msg.data.resize(audio_msg.nch * msg->len);

    Map<ArrayXXf> audio_out((float*)audio_msg.data.data(), audio_msg.nch, audio_msg.len);
    for (int m=0;m<list_ch.size();m++) {
      audio_out.row(m) = audio_in.row(list_ch[m]);
    }

    audio_pub.publish(audio_msg);
  }
};

int main(int argc, char *argv[])
{
  Eigen::initParallel();

  ChannelSelectorNode mpn(argc, argv);
  ros::spin();

  return 0;
}
