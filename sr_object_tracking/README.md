## sr_object_tracking

This package contains a base class that initializes the tracking node. The implementation specific classes contains the different tracking algorithms (Camshift and an improved one). These ones work using a track_box, supplied by the other sr_vision nodes (sr_gui_servoing or sr_object_segmentation).

Two launch files are available, one for a Kinect usage and the other for UVC cameras. The first one uses the `freenect` driver, the second one uses `libuvc`.


Furthermore, algorithms are based upon OpenCV, that could be installed with :

`sudo apt-get install libopencv-dev python-opencv`

