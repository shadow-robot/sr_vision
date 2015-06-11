## sr_object_tracking

This package contains a base class that init the tracking node, let the user select a region of interest (drag&drop) and launch the tracking. The implementation specific classes contain the different tracking algorithms (only Camshit for now).

The executable is tracking.py, localised in the "scripts/" folder. Working needs a topic with the camera image. For example Freenect_launch, publishing in "/camera/rgb/image_color", gives the image from the Kinect.

Furthermore, Camshift is based upon OpenCV, that could be installed with :

`sudo apt-get install libopencv-dev python-opencv`

