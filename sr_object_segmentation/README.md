##sr_object_segmentation
This package contains a base class that gets the data, etc... and implementation specific classes with the different segmentation algorithms.
The BlobsSegmentation algorithm is based upon SimpleCV (see below for the installation instuctions).

######################################################

#                Pillow INSTALLATION              

######################################################

See the instructions on the website :
http://pillow.readthedocs.org/en/latest/installation.html


The easiest way to install Pillow is with pip :
$ sudo pip install Pillow

Can also be installed with easy-install :
$ easy_install Pillow

Or directy with the source :
download and extract the compressed archive from PyPI (https://pypi.python.org/pypi/Pillow) and inside it run:
$ python setup.py install



######################################################

#                Simple CV INSTALLATION              

######################################################

The easiest way to install SimpleCV is with the packages for your distribution (Windows, Mac, Linux) included on the website (http://www.simplecv.org). Although it is tested on many platforms there maybe scenarios where it just won't work with the package installer. Below is instructions on how to install in this case.


### Ubuntu 12.04

##### Install with pip

sudo apt-get install ipython python-opencv python-scipy python-numpy python-pygame python-setuptools python-pip
sudo pip install https://github.com/sightmachine/SimpleCV/zipball/develop

##### Install using clone of SimpleCV repository

sudo apt-get install ipython python-opencv python-scipy python-numpy python-pygame python-setuptools git
git clone https://github.com/sightmachine/SimpleCV.git
cd SimpleCV/
sudo pip install -r requirements.txt
sudo python setup.py install

then just run 'simplecv' from the shell.

##############################################################
