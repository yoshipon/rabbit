#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <alsa/asoundlib.h>
#include <Eigen/Dense>

#include <ros/ros.h>

#include <rabbit_msgs/Audio.h>
#include <rabbit_common/Playback.h>

using namespace std;
using namespace Eigen;

#include "rabbit_common/alsa_audio.hpp"

class AlsaNode
{
protected:
  string in_device_name, out_device_name;

  int n_mic, n_spk;

  int n_step, n_frame;
  int n_periods;

  snd_pcm_uframes_t n_alsa_buffer;

  AlsaAudio alsa_in, alsa_out;

  thread th_in, th_out;

  mutex playback_mtx;
  bool playback_requested;

  mutex sync_recording_mtx;
  condition_variable sync_recording_cv;
  bool sync_recording_requested;
  MatrixXi sync_recording;
  int sync_recording_idx;

  rabbit_msgs::Audio audio_msg;

  ros::Publisher audio_pub;

  ros::ServiceServer playback_srv;

  void initialize_input()
  {
    alsa_in.open(in_device_name.c_str(), SND_PCM_STREAM_CAPTURE);

    alsa_in.set(snd_pcm_hw_params_set_access, SND_PCM_ACCESS_RW_INTERLEAVED);
    alsa_in.set(snd_pcm_hw_params_set_format, SND_PCM_FORMAT_S24_LE);
    alsa_in.set(snd_pcm_hw_params_set_rate, 16000, 0);
    alsa_in.set(snd_pcm_hw_params_set_channels, n_mic);
    alsa_in.set(snd_pcm_hw_params_set_periods, n_periods, 0);
    alsa_in.set(snd_pcm_hw_params_set_buffer_size_near, &n_alsa_buffer);
    alsa_in.prepare();
  }

  void initialize_output()
  {
    alsa_out.open(out_device_name.c_str(), SND_PCM_STREAM_PLAYBACK);

    alsa_out.set(snd_pcm_hw_params_set_access, SND_PCM_ACCESS_RW_INTERLEAVED);
    alsa_out.set(snd_pcm_hw_params_set_format, SND_PCM_FORMAT_S24_LE);
    alsa_out.set(snd_pcm_hw_params_set_rate, 16000, 0);
    alsa_out.set(snd_pcm_hw_params_set_channels, n_spk);
    alsa_out.set(snd_pcm_hw_params_set_periods, n_periods, 0);
    alsa_out.set(snd_pcm_hw_params_set_buffer_size_near, &n_alsa_buffer);
    alsa_out.prepare();
  }

public:
  AlsaNode(int argc, char *argv[])
  {
    // initialize ros node
    ros::init(argc, argv, "alsa");
    ros::NodeHandle n;

    // load parameters
    ros::param::param<std::string>("~in_device_name" ,  in_device_name, "plughw:2,0");
    ros::param::param<std::string>("~out_device_name", out_device_name, "plughw:3,0");

    ros::param::param<int>("~n_mic", n_mic, 2);
    ros::param::param<int>("~n_spk", n_spk, 1);

    ros::param::param<int>("~n_step", n_step, 160);
    ros::param::param<int>("~n_frame", n_frame, 512);
    ros::param::param<int>("~n_periods", n_periods, 3);

    int tmp;
    ros::param::param<int>("~n_alsa_buffer", tmp, 2048);
    n_alsa_buffer = tmp;

    // advertize messages
    audio_pub = n.advertise<rabbit_msgs::Audio>("audio", 100);

    // print parameters
    ROS_INFO_STREAM("Input Device Name: "       <<  in_device_name);
    ROS_INFO_STREAM("Output Device Name: "      << out_device_name);
    ROS_INFO_STREAM("# of mics.: "  << n_mic);
    ROS_INFO_STREAM("# of spks.: " << n_spk);
    ROS_INFO_STREAM("Step length: "       << n_step);
    ROS_INFO_STREAM("Frame length: "      << n_frame);
    ROS_INFO_STREAM("# of periods: "      << n_periods);
    ROS_INFO_STREAM("# of alsa buffer: " << n_alsa_buffer);

    // initialize messages
    audio_msg.header.frame_id = "audio";
    audio_msg.nch = n_mic;
    audio_msg.len = n_frame;
    audio_msg.data.resize(n_frame * n_mic);

    initialize_input();
    sync_recording_requested = false;

    if (n_spk > 0) {
      initialize_output();

      playback_srv = n.advertiseService("playback", &AlsaNode::playback, this);
      ROS_INFO_STREAM("Ready to playback.");

      playback_requested = false;

      th_out = thread(&AlsaNode::playback_silent, this);
    }

    th_in = thread(&AlsaNode::capture, this);
  }

  void capture()
  {
    int n_buf = (n_frame / n_step + 1) * n_step;
    MatrixXi buf(n_mic, n_buf);
    int idx_buf = 0;

    Map<MatrixXf> map_mic(  audio_msg.data.data(),     n_mic,    n_frame);

    VectorXi imu_offsets = VectorXi::Constant(8, -10);

    for (int seq=0;ros::ok();seq++) {
      // read from audio device.
      alsa_in.read(buf.block(0, idx_buf, n_mic, n_step), n_step);
      ros::Time stamp = ros::Time::now();
      
      // capture microphone data
      if (n_mic > 0) {
	int tmp = idx_buf + n_step - n_frame;

	for (int m=0;m<n_mic;m++) {
	  if (tmp < 0) {
	    map_mic.block(m,    0, 1,           -tmp) = buf.block(m, n_buf+tmp, 1,           -tmp)
	      .template cast<float>() / 0x007fffff;
	    map_mic.block(m, -tmp, 1, idx_buf+n_step) = buf.block(m,         0, 1, idx_buf+n_step)
	      .template cast<float>() / 0x007fffff;
	  } else {
	    map_mic.block(m,    0, 1,        n_frame) = buf.block(m,       tmp, 1,        n_frame)
	      .template cast<float>() / 0x007fffff;
	  }
	}

	audio_msg.header.seq = seq;
	audio_msg.header.stamp = stamp;
	audio_pub.publish(audio_msg);

	if (sync_recording_requested) {
	  if (sync_recording_idx < sync_recording.cols()) {
	    for (int m=0;m<n_mic;m++) {
	      sync_recording.block(m, sync_recording_idx, 1, n_step) =
		buf.block(m, idx_buf, 1, n_step);
	    }
	    sync_recording_idx += n_step;
	  } else {
	    lock_guard<mutex> lock(sync_recording_mtx);
	    sync_recording_requested = false;
	    sync_recording_cv.notify_all();
	  }
	}
      }

      // increment idx_buf
      idx_buf += n_step;
      if (idx_buf >= n_buf) idx_buf = 0;
    }
  }

  void playback_silent()
  {
    MatrixXi frame = MatrixXi::Zero(n_spk, n_step);
    while (ros::ok() && !playback_requested) {
      alsa_out.write(frame, n_step);
    }
  }

  bool playback(rabbit_common::Playback::Request& req, rabbit_common::Playback::Response& res) 
  {
    if (req.playback_data.nch != n_spk) {
      return false;
    }

    lock_guard<mutex> lock(playback_mtx);

    ROS_INFO_STREAM("Playback request received");

    // Stop playback silence
    playback_requested = true;
    th_out.join();

    // Prepare sync recording
    sync_recording.resize(n_mic, (req.playback_data.len / n_step + 1) * n_step);
    sync_recording_idx = 0;

    // Playback requested data
    Map<MatrixXf> req_data(req.playback_data.data.data(), n_spk, req.playback_data.len);
    MatrixXi frame(n_spk, n_step);
    int t_ = 0;
    for (int t=0;t<req.playback_data.len - n_step && ros::ok();t+=n_step) {
      if (t > n_alsa_buffer && !sync_recording_requested) {
	sync_recording_requested  = true;
      }

      frame = (req_data.block(0, t, req.playback_data.nch, n_step) * 0x007fffff).cast<int>();
      alsa_out.write(frame, n_step);
    }

    // back to playback silence
    playback_requested = false;
    th_out = thread(&AlsaNode::playback_silent, this);
    
    // wait for finishing the recording
    {
      unique_lock<mutex> lock(sync_recording_mtx);
      sync_recording_cv.wait(lock, [&]{ return !sync_recording_requested; });
    } 

    // setting the results
    res.captured_data.header.frame_id = "sync_recording_audio";
    res.captured_data.nch = n_mic;
    res.captured_data.len = sync_recording.cols();
    res.captured_data.data.resize(n_mic * sync_recording.cols());
    Map<MatrixXf>(res.captured_data.data.data(), n_mic, sync_recording.cols()) = sync_recording.cast<float>() / 0x007fffff;

    ROS_INFO_STREAM("Playback is done");

    return true;
  }

  ~AlsaNode()
  {
    th_in.join();
    th_out.join();
  }
};

int main(int argc, char *argv[])
{
  AlsaNode raspzx(argc, argv);
  ros::spin();

  return 0;
}
