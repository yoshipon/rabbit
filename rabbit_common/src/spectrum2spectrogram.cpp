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

using ArrayXXXf  = Array<ArrayXXf, Dynamic, 1>;
using ArrayXXXcf = Array<ArrayXXcf, Dynamic, 1>;

class MedianPostfilterNode
{
  ros::Subscriber spectrum_sub;
  ros::Publisher spectrogram_pub;

  int n_ch, n_freq, n_batch;

  rabbit_msgs::Spectrogram spectrogram_msg;

  int t;

public:
  MedianPostfilterNode(int argc, char *argv[])
  {
    ros::init(argc, argv, "spectrogram2spectrum");
    ros::NodeHandle n;

    ros::param::param<int>(   "~n_ch",    n_ch,   8);
    ros::param::param<int>( "~n_freq",  n_freq, 513);
    ros::param::param<int>("~n_batch", n_batch, 200);

    ROS_INFO_STREAM("# of channels: "         <<   n_ch);
    ROS_INFO_STREAM("# of freq. bins: "    <<  n_freq);
    ROS_INFO_STREAM("Batch frqme length: " << n_batch);

    spectrogram_pub = n.advertise<rabbit_msgs::Spectrogram>("spectrogram", 10);
    spectrum_sub = n.subscribe("spectrum", 300, &MedianPostfilterNode::spectrum_callback, this);

    // spec = ArrayXXXcf::Constant(n_mic, ArrayXXcf::Zero(n_freq, n_batch));

    spectrogram_msg.nch = n_ch;
    spectrogram_msg.nfreq = n_freq;
    spectrogram_msg.len = n_batch;
    spectrogram_msg.data.resize(2 * n_ch * n_freq * n_batch);

    t = 0;
  }

  void spectrum_callback(const rabbit_msgs::Spectrum::ConstPtr &msg)
  {
    Map<ArrayXXcf> spec_((complex<float>*)msg->data.data(), msg->nfreq, msg->nch);

    for (int m=0;m<n_ch;m++) {
      Map<ArrayXXcf>((complex<float>*)&spectrogram_msg.data[2 * m * n_freq * n_batch], n_freq, n_batch).col(t) = spec_.col(m);
    }

    t++;

    if (t == n_batch) {
      spectrogram_msg.header = msg->header;
      spectrogram_pub.publish(spectrogram_msg);

      t = 0;
    }
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
