# Rabbit
ROS packages for multi-channel audio signal processings

## Package Descriptions
* `rabbit_common`: basic functionarities such as captureing, playbacking, and saving audio signals.
* `rabbit_msgs`: basic messages for Rabit.
* `rabbit_stft`: STFT and iSTFT that use FFTW.
* `rabbit_rqt_plugins`: visualization plugins for rqt.
* `rabbit_hark_bridge`: interface to HARK (currently TDB).

## Installing
Install dependencies:

    sudo apt-get install libfftw3-dev libeigen3-dev libboost-all-dev libasound2-dev 

If you want to use HARK interface, install `hark-ros-indigo` and `hark-ros-stacks-indigo`:

    sudo apt-get install hark-ros-indigo and hark-ros-stacks-indigo

Download the packages into `catkin_ws/src`:

    cd ~/catkin_ws/src
    git clone https://github.com/yoshipon/rabbit.git

Compile them:

    cd ~/catkin_ws
    catkin_make


## License
* `rabbit_stft`: GPL
* `other packages`: MIT
