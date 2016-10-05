#include <iostream>
#include <fstream>
#include <string>
#include <complex>

#include <boost/format.hpp>

#include <Eigen/Dense>


#include <ros/ros.h>

#include <rabbit_msgs/Spectrogram.h>
#include <rabbit_msgs/Spectrum.h>

using namespace Eigen;
using namespace std;

class MedianPostfilterNode
{
  ros::Subscriber spectrogram_sub;
  ros::Publisher spectrum_pub;

public:
  MedianPostfilterNode(int argc, char *argv[])
  {
    ros::init(argc, argv, "spectrogram2spectrum");
    ros::NodeHandle n;

    spectrum_pub = n.advertise<rabbit_msgs::Spectrum>("spectrum", 300);
    spectrogram_sub = n.subscribe("spectrogram", 300, &MedianPostfilterNode::spectrogram_callback, this);

  }

  void spectrogram_callback(const rabbit_msgs::Spectrogram::ConstPtr &msg)
  {
    int F = msg->nfreq, T = msg->len, M = msg->nch;

    rabbit_msgs::Spectrum spec_msg;
    spec_msg.nch = M;
    spec_msg.nfreq = F;
    spec_msg.data.resize(2 * F * M);
    Map<ArrayXXcf> spec((complex<float>*)spec_msg.data.data(), F, M);
    for (int t=0;t<T;t++) {
      spec = Map<const ArrayXXcf>((complex<float>*)msg->data.data() + t * F * M, F, M);

      spectrum_pub.publish(spec_msg);
    }
    
    // int n_freq, n_batch;
  


    // Map<ArrayXXcf> spec_out((complex<float>*)enhanced_spectrogram_msg.data.data(),
    // 			    n_freq, n_batch);
    // spec_out = enhanced_amp_spec * spec / amp_spec;

    // enhanced_spectrogram_pub.publish(enhanced_spectrogram_msg);
  }
};

int main(int argc, char *argv[])
{
  Eigen::initParallel();
  srand((unsigned int) time(0));

  MedianPostfilterNode mpn(argc, argv);
  ros::spin();

  return 0;
}
