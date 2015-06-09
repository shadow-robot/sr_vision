##sr_object_tracking

This package contains a base class that init the tracking node, let the user select a region of interest (drag&drop) and launch the tracking. The implementation specific classes contain the different tracking algorithms (only Camshit for now).

The executable is tracking.py, localised in the "scripts/" folder. Working needs a topic with the camera image. For example Freenect_launch, publishing in "/camera/rgb/image_color", gives the image from the Kinect.

Furthermore, Camshift is based upon OpenCV (see below for the installation instuctions).



#### OpenCV INSTALLATION 

To install OpenCV 2.4.9 on the Ubuntu 14.04 operating system, first install a developer environment to build OpenCV.

##### Installation

Install dependencies(support for reading and writing image files, drawing on the screen, some needed tools, other libraries, etc…)

```sudo apt-get install build-essential libgtk2.0-dev libjpeg-dev libtiff4-dev libjasper-dev libopenexr-dev cmake python-dev python-numpy python-tk libtbb-dev libeigen3-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev libqt4-dev libqt4-opengl-dev sphinx-common texlive-latex-extra libv4l-dev libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev default-jdk ant libvtk5-qt4-dev```

Get the OpenCV 2.4.9 source code:

```wget http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.9/opencv-2.4.9.zip
unzip opencv-2.4.9.zip
cd opencv-2.4.9```

Generate the Makefile by using cmake:

```mkdir build
cd build
cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON ..```

Compile and install OpenCV 2.4.9:
```make
sudo make install```


##### Configuration

OpenCV has to be configured. First, open the opencv.conf file with the following code:

`sudo gedit /etc/ld.so.conf.d/opencv.conf`

Add the following line at the end of the file(it may be an empty file, that is ok) and then save it:

`/usr/local/lib`

Run the following code to configure the library:

`sudo ldconfig`

Open another file:

`sudo gedit /etc/bash.bashrc`

Add these two lines at the end of the file and save it:

```PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
export PKG_CONFIG_PATH```

Finally, close the console and open a new one, restart the computer or logout and then login again. OpenCV will not work correctly until doing this.


