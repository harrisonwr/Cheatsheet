Install OpenCV on Ubuntu 14.04 64 bit
===
### Install Require Package
```python
# Tutorial Links:

# 1. Installation
# http://www.bogotobogo.com/OpenCV/opencv_3_tutorial_ubuntu14_install_cmake.php

# 2. Change from ffmpeg package to libav-tools package
# http://askubuntu.com/questions/432542/is-ffmpeg-missing-from-the-official-repositories-in-14-04

# 3. (optional guide)
# http://rodrigoberriel.com/2014/10/installing-opencv-3-0-0-on-ubuntu-14-04/

$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get install libopencv-dev build-essential checkinstall cmake pkg-config yasm libtiff4-dev libjpeg-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev python-dev python-numpy libtbb-dev libqt4-dev libgtk2.0-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils libav-tools
```

### Getting OpenCV source code from the Git Repository
```python
# you can create OpenCV directory wherever you want
$ mkdir OpenCV
$ cd OpenCV
$ git clone https://github.com/Itseez/opencv.git
```

### Building OpenCV
```python
# Tutorial for encountering error about ICV download failed:
# http://answers.opencv.org/question/40425/opencv-build-fails-because-i-cannot-download-icv-on-our-build-farm/
# You could download the ipccv package manually and add it to the 3rd party
# folder OR you could simply build with cmake flag -D WITH_IPP=OFF
# addition: -D WITH_OPENNI=ON

$ cd opencv
$ mkdir release
$ cd release
$ cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_IPP=OFF -D WITH_OPENNI=ON ..
```
##### Make and Install It
```python
$ make
$ sudo make install
```
##### Configure the System Wide Library Search Path
```python
$ sudo /bin/bash -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
$ sudo ldconfig
```

### Test It Out
#### Running a OpenCV Sample
#####  Compile the Samples
```python
$ cd opencv/opencv/samples/
$ sudo cmake .
$ sudo make -j $(nproc)
```
##### Run a Sample
```python
cd cpp/
./cpp-example-facedetect ../data/lena.jpg
./cpp-example-houghlines ../data./pic1.png
```
