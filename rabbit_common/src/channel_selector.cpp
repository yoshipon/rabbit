#include <iostream>
#include <fstream>
#include <string>
#include <complex>
#include <random>
#include <cmath>

#include <boost/format.hpp>

#include <Eigen/Dense>

#include <sndfile.hh>
#include <fftw3.h>

#include <ros/ros.h>

#include <rabbit_msgs/Spectrum.h>

using namespace Eigen;
using namespace std;

class ChannelSelectorNode
{
  ros::Subscriber spec_sub;
  ros::Publisher spec_pub;

  rabbit_msgs::Spectrum spec_msg;

  vector<int> list_ch;
  
public:
  ChannelSelectorNode(int argc, char *argv[])
  {
    ros::init(argc, argv, "channel_selector");
    ros::NodeHandle n;

    ros::param::param<vector<int>>("~ch_list", list_ch, {0});

    spec_pub = n.advertise<rabbit_msgs::Spectrum>("spec_out", 100);
    spec_sub = n.subscribe("spec_in", 100, &ChannelSelectorNode::spectrogram_callback, this);

    spec_msg.nch = list_ch.size();
  }

  void spectrogram_callback(const rabbit_msgs::Spectrum::ConstPtr &msg)
  {
    Map<const ArrayXXcf> spec_in((complex<float>*)msg->data.data(), msg->nfreq, msg->nch);

    spec_msg.nfreq = msg->nfreq;
    spec_msg.data.resize(2 * spec_msg.nch * msg->nfreq);

    Map<ArrayXXcf> spec_out((complex<float>*)spec_msg.data.data(), spec_msg.nfreq, spec_msg.nch);
    for (int m=0;m<list_ch.size();m++) {
      spec_out.col(m) = spec_in.col(list_ch[m]);
    }

    spec_pub.publish(spec_msg);
  }
};

int main(int argc, char *argv[])
{
  Eigen::initParallel();
  srand((unsigned int) time(0));

  ChannelSelectorNode mpn(argc, argv);
  ros::spin();

  return 0;
}
