#!/usr/bin/env python
import sys

from sr_object_tracking.sr_object_tracking import run
from sr_object_tracking.camshift import CamshiftTracking
from sr_object_tracking.simplecv_find_template import SimpleCV_FindTemplate


def main(args):
    """
    Process the tracing with one of the available algorithms
    """
    try:
        algos = [CamshiftTracking, SimpleCV_FindTemplate]
        run(algos[0])
    except KeyboardInterrupt:
        print "Shutting down tracking node."


if __name__ == '__main__':
    main(sys.argv)
