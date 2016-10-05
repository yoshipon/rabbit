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
  ros::Subscriber spectrogram_sub;
  ros::Publisher enhanced_spectrogram_pub;

  rabbit_msgs::Spectrogram enhanced_spectrogram_msg;

  int n_freq, n_batch;
  int width, height;

  ArrayXXf amp_spec, enhanced_amp_spec;
  
public:
  ChannelSelectorNode(int argc, char *argv[])
  {
    ros::init(argc, argv, "svb_mrnmf");
    ros::NodeHandle n;

    // basic parameters
    ros::param::param<int>( "~n_freq",  n_freq, 257);
    ros::param::param<int>("~n_batch", n_batch, 200);

    ros::param::param<int>( "~width",  width, 5);
    ros::param::param<int>("~height", height, 3);

    amp_spec = ArrayXXf::Zero(n_freq, n_batch);
    enhanced_amp_spec = ArrayXXf::Zero(n_freq, n_batch);

    enhanced_spectrogram_pub = n.advertise<rabbit_msgs::Spectrogram>("postfiltered_spectrogram", 10);
    spectrogram_sub = n.subscribe("spectrogram", 2 * n_batch, &ChannelSelectorNode::spectrogram_callback, this);

    enhanced_spectrogram_msg.nch = 1;
    enhanced_spectrogram_msg.nfreq = n_freq;
    enhanced_spectrogram_msg.len = n_batch;
    enhanced_spectrogram_msg.data.resize(2 * n_freq * n_batch);
  }

  void spectrogram_callback(const rabbit_msgs::Spectrogram::ConstPtr &msg)
  {
    Map<const ArrayXXcf> spec((complex<float>*)msg->data.data(), msg->nfreq, msg->len);
    amp_spec = spec.abs();

    median_filter(amp_spec, enhanced_amp_spec, height, width);

    Map<ArrayXXcf> spec_out((complex<float>*)enhanced_spectrogram_msg.data.data(),
			    n_freq, n_batch);
    spec_out = enhanced_amp_spec * spec / (amp_spec + EPS);

    enhanced_spectrogram_pub.publish(enhanced_spectrogram_msg);
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
