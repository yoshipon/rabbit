#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <cstdlib>

#include <Eigen/Dense>
#include <fftw3.h>

#include <ros/ros.h>

#include "rabbit_msgs/Audio.h"
#include "rabbit_msgs/Spectrum.h"

using namespace std;
using namespace Eigen;

class STFTNode
{
protected:
  int n_ch, n_step, n_frame;

  ros::Subscriber rabbit_sub;
  ros::Publisher spec_pub;

  rabbit_msgs::Spectrum spectrum_msg;

  ArrayXXf hamming;

public:
  STFTNode(int argc, char *argv[])
  {
    ros::init(argc, argv, "stft", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    // load parameters
    ros::param::param<int>("~n_ch", n_ch, 8);
    ros::param::param<int>("~n_step", n_step, 160);
    ros::param::param<int>("~n_frame", n_frame, 512);
 
    hamming = ArrayXXf::Zero(1, n_frame);
    for (int n=0;n<n_frame;n++) {
      hamming(n) = 0.54 - 0.46 * cos(2 * M_PI * (float)n / n_frame);
    }
    
    // configure topics & servers
    spectrum_msg.nch = n_ch;
    spectrum_msg.nfreq = n_frame / 2 + 1;
    spectrum_msg.data.resize(2 * n_ch * spectrum_msg.nfreq);

    spec_pub = n.advertise<rabbit_msgs::Spectrum>("spectrum", 50);
    rabbit_sub = n.subscribe("audio", 50, &STFTNode::rabbit_callback, this);
  }

  void rabbit_callback(const rabbit_msgs::Audio::ConstPtr &audio)
  {
    if (n_ch != audio->nch or n_frame != audio->len) {
      ROS_ERROR_STREAM("Invalid #channels or buffer size");
      return;
    }

    Map<const ArrayXXf> wav(audio->data.data(), n_ch, n_frame);

    Map<ArrayXXcf> spec((complex<float>*)spectrum_msg.data.data(), spectrum_msg.nfreq, n_ch);
    ArrayXXf fr(1, n_frame);
    for (int m=0;m<n_ch;m++) {
      fr = wav.row(m) * hamming;

      fftwf_plan plan_fwd = fftwf_plan_dft_r2c_1d(
        n_frame, fr.data(), (fftwf_complex*)spec.col(m).data(), FFTW_ESTIMATE);
      fftwf_execute(plan_fwd);
      fftwf_destroy_plan(plan_fwd);
    }

    spectrum_msg.header.seq   = audio->header.seq;
    spectrum_msg.header.stamp = audio->header.stamp;

    spec_pub.publish(spectrum_msg);
  }
};


int main(int argc, char *argv[])
{

  STFTNode stft(argc, argv);
  ros::spin();

  return 0;
}
