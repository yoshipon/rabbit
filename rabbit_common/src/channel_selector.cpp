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
  
public:
  ChannelSelectorNode(int argc, char *argv[])
  {
    ros::init(argc, argv, "channel_selector");
    ros::NodeHandle n;

    spec_pub = n.advertise<rabbit_msgs::Spectrum>("spec_in", 100);
    spec_sub = n.subscribe("spec_out", 100, &ChannelSelectorNode::spectrogram_callback, this);
  }

  void spectrogram_callback(const rabbit_msgs::Spectrum::ConstPtr &msg)
  {
    // Map<const ArrayXXcf> spec((complex<float>*)msg->data.data(), msg->nfreq, msg->len);
    // amp_spec = spec.abs();

    // median_filter(amp_spec, enhanced_amp_spec, height, width);

    // Map<ArrayXXcf> spec_out((complex<float>*)enhanced_spectrogram_msg.data.data(),
    // 			    n_freq, n_batch);
    // spec_out = enhanced_amp_spec * spec / (amp_spec + EPS);

    // enhanced_spectrogram_pub.publish(enhanced_spectrogram_msg);
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
