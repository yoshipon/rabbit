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

class ISTFTNode
{
protected:
  int n_ch, n_step, n_frame;

  ros::Subscriber spec_sub;
  ros::Publisher rabbit_pub;

  rabbit_msgs::Audio rabbit_msg;

  ArrayXXf buf;
  ArrayXXf win_buf;
  int buf_idx;

  ArrayXXf hamming;

public:
  ISTFTNode(int argc, char *argv[])
  {
    ros::init(argc, argv, "istft", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    // load parameters
    ros::param::param<int>("~n_ch", n_ch, 8);
    ros::param::param<int>("~n_step", n_step, 160);
    ros::param::param<int>("~n_frame", n_frame, 512);

    // configure buffer
    buf     = ArrayXXf::Zero(n_ch, 2*n_frame);
    win_buf = ArrayXXf::Zero(   1, 2*n_frame);
    buf_idx = 0;
 
    hamming = ArrayXXf::Zero(1, n_frame);
    for (int n=0;n<n_frame;n++) {
      hamming(n) = 0.54 - 0.46 * cos(2 * M_PI * (float)n / n_frame);
    }
    // hamming2 = hamming2.square();
    
    // configure topics & servers
    rabbit_msg.nch = n_ch;
    rabbit_msg.len = n_frame;
    rabbit_msg.data.resize(n_ch * rabbit_msg.len);

    rabbit_pub = n.advertise<rabbit_msgs::Audio>("audio", 400);

    spec_sub = n.subscribe("spectrum", 400, &ISTFTNode::spectrum_callback, this);
  }

  template<typename Any>
  void set_rbuf(ArrayXXf &buf, const ArrayBase<Any> &data, int idx, int ch)
  {
    int B = buf.cols();
    idx = idx < 0 ? idx % B + B : idx % B;
    if (idx + data.cols() <= B) {
      buf.block(ch, idx, data.rows(),     data.cols()) = data;
    } else {
      int l = B - idx;
      buf.block(ch, idx, data.rows(),             l) = data.block(0, 0, data.rows(),             l);
      buf.block(ch,   0, data.rows(), data.cols()-l) = data.block(0, l, data.rows(), data.cols()-l);
    }
  }


  template<typename Any>
  void add_rbuf(ArrayXXf &buf, const ArrayBase<Any> &data, int idx, int ch)
  {
    int B = buf.cols();
    idx = idx < 0 ? idx % B + B : idx % B;
    if (idx + data.cols() <= B) {
      buf.block(ch, idx, data.rows(),     data.cols()) += data;
    } else {
      int l = B - idx;
      buf.block(ch, idx, data.rows(),             l) += data.block(0, 0, data.rows(),             l);
      buf.block(ch,   0, data.rows(), data.cols()-l) += data.block(0, l, data.rows(), data.cols()-l);
    }
  }

  // [TODO]
  ArrayXXf get_rbuf(ArrayXXf &buf, int idx, int cols)
  {
    ArrayXXf ret(buf.rows(), cols);

    int B = buf.cols();
    idx = idx < 0 ? idx % B + B : idx % B;
    if (idx + cols <= B) {
      ret = buf.block(0, idx, buf.rows(), cols);
    } else {
      int l = B - idx;
      ret.block(0, 0, buf.rows(),      l) = buf.block(0, idx, buf.rows(),      l);
      ret.block(0, l, buf.rows(), cols-l) = buf.block(0,   0, buf.rows(), cols-l);
    }

    return ret;
  }

  void spectrum_callback(const rabbit_msgs::Spectrum::ConstPtr &spectrum)
  {
    if (n_ch != spectrum->nch or n_frame / 2 + 1 != spectrum->nfreq) {
      ROS_ERROR_STREAM("Invalid #channels or buffer size");
      return;
    }

    set_rbuf(win_buf, ArrayXXf::Zero(1, n_step), buf_idx + n_frame - n_step, 0);
    add_rbuf(win_buf,         hamming * hamming,                    buf_idx, 0);

    Map<const ArrayXXcf> spec((complex<float>*)spectrum->data.data(), spectrum->nfreq, spectrum->nch);
    ArrayXXf fr(1, n_frame);
    set_rbuf(buf, ArrayXXf::Zero(n_ch, n_step), buf_idx + n_frame - n_step, 0);
    for (int m=0;m<n_ch;m++) {
      fftwf_plan plan_bwd = fftwf_plan_dft_c2r_1d(n_frame, (fftwf_complex*)spec.col(m).data(),
						  fr.data(), FFTW_ESTIMATE);
      fftwf_execute(plan_bwd);
      fftwf_destroy_plan(plan_bwd);

      add_rbuf(buf, fr * hamming, buf_idx, m);
    }

    rabbit_msg.header.seq = spectrum->header.seq;
    rabbit_msg.header.stamp = spectrum->header.stamp;
    Map<ArrayXXf>(rabbit_msg.data.data(), n_ch, n_frame) = get_rbuf(buf, buf_idx - n_frame, n_frame) /
      get_rbuf(win_buf, buf_idx - n_frame, n_frame).replicate(n_ch, 1) / n_frame;

    rabbit_pub.publish(rabbit_msg);

    buf_idx += n_step;
    if (buf_idx >= buf.cols()) buf_idx -= buf.cols();
  }
};

int main(int argc, char *argv[])
{

  ISTFTNode istft(argc, argv);
  ros::spin();

  return 0;
}
