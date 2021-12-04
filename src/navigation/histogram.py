import numpy as np
from utils.geom import getHeading

class Histogram():
    """
    Histogram inner class used for managing bins.

    There are 3 types of bins: in x axis, in y axis and in theta
    axis. The bins can be thought of as cells for a volume where
    particles belong to based on their parameters being within a
    specific interval.
    """
    def __init__(self, width, height, origin, num_bins=(400, 400, 1)):
        """Initializes the histogram object.

        Args:
            width (float): the width of the map
            height (float): the height of the map
            origin ()
            num_bins (tuple): the number of bins in each dimension
        """
        # Number of bins in each dimension
        size_x, size_y, size_theta = num_bins

        # Get size intervals
        self.x_bins = np.linspace(origin.x, origin.x + width, size_x + 1)
        self.y_bins = np.linspace(origin.y, origin.y + height, size_y + 1)
        self.theta_bins = np.linspace(-np.pi, np.pi, size_theta + 1)

        # Keep track of requests for particles belonging to bins
        self.non_empty_bins = []


    def add_if_empty(self, pose):
        """
        Adds a bin to a `non_empty_bins` list if passed pose belongs
        to that bin.
            
        Args:
            pose (geometry_msgs.msg.Pose): the particle pose
        Return:
            (bool): `True` if bin is empty, `False` otherwise
        """
        # Get x, y and theta
        x = pose.position.x
        y = pose.position.y
        theta = getHeading(pose.orientation)
        
        # Find the corresponding bin (interval)
        bin = (np.searchsorted(self.x_bins, x), 
               np.searchsorted(self.y_bins, y),
               np.searchsorted(self.theta_bins, theta))
            
        # Add to non-empty list if such bin is not there
        if bin in self.non_empty_bins:
            return False
        else:
            self.non_empty_bins.append(bin)
            return True