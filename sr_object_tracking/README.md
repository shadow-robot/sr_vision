##sr_object_tracking
This package contains a base class that init the tracking node, let the user select a region of interest (drag&drop) and launch the tracking. The implementation specific classes contain the different tracking algorithms (only Camshit for now).

The executable is tracking.py, localised in the "scripts/" folder. Working needs a topic with the camera image. For example Freenect_launch, publishing in "/camera/rgb/image_color", gives the image from the Kinect.

Furthermore, Camshift is based upon OpenCV (see below for the installation instuctions).



######################################################

#                OpenCV INSTALLATION              

######################################################

#### Ubuntu 12.04

#To install OpenCV 2.4.2 or 2.4.3 on the Ubuntu 12.04 operating system, first install a developer environment to build OpenCV.
sudo apt-get -y install build-essential cmake pkg-config
#Install Image I/O libraries
sudo apt-get -y install libjpeg62-dev 
sudo apt-get -y install libtiff4-dev libjasper-dev
#Install the GTK dev library
sudo apt-get -y install  libgtk2.0-dev
#Install Video I/O libraries
sudo apt-get -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev

#Optional - install support for Firewire video cameras
sudo apt-get -y install libdc1394-22-dev
#Optional - install video streaming libraries
sudo apt-get -y install libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev 
#Optional - install the Python development environment and the Python Numerical library
sudo apt-get -y install python-dev python-numpy
#Optional - install the parallel code processing library (the Intel tbb library)
sudo apt-get -y install libtbb-dev
#Optional - install the Qt dev library
sudo apt-get -y install libqt4-dev

#Now download OpenCV 2.4 to wherever you want to compile the source.
mkdir xxx
cd xxx 
wget http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.2/Op...
#or 
wget http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.3/Op...
tar -xvf OpenCV-2.4.*.tar.bz2
#Create and build directory and onfigure OpenCV with cmake. Don't forget the .. part at the end of cmake cmd 
cd OpenCV-2.4.*
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local
-D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON 
-D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON 
-D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON ..
#Now compile it
make
#And finally install OpenCV
sudo make install

##############################################################
