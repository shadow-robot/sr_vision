##sr_object_segmentation
This package contains a base class that gets the data, etc... and implementation specific classes with the different segmentation algorithms. The most advanced is color and shape based (see below for the custom ones creation).

###Adding custom shapes
A dataset of shapes is already present in the [shapes](shapes/dataset) directory : circle, rectangle, star, strawberry, banana and leaf. 

Custom ones can be added using the [shape_creation](shapes/shape_creation.py) file and an appropriate image. This last one has to represent only the shape, and the result is better without too much noise. The result is displayed for control.

The matching is controlled with a threshold parameter which can be set in the launchfile (shape_threshold). Notice that lower is more restrictive.


####Usage
`python shape_creation.py -n [name] -i [image file path]`

Finally, the new shape description file (.npy) has to be specified as parameter in the [launch file](../sr_object_tracking/launch/tracking.launch)

