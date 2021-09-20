"""
Wrapper module providing a simple cartesian point mapping.
"""
from dataclasses import dataclass


@dataclass
class CartesianPoint:
    """
    Each cartesian point has x and y coordinates.
    """

    position_x: float = 0
    position_y: float = 0

    def __iter__(self):
        yield self.position_x
        yield self.position_y

    @property
    def x(self):
        """
        Alternative naming for retrieving the x-coordinate.
        """
        # pylint: disable=invalid-name
        return self.position_x

    @property
    def y(self):
        """
        Alternative naming for retrieving the y-coordinate.
        """
        # pylint: disable=invalid-name
        return self.position_y


@dataclass
class LidarPoint:
    """
    Each lidar point has an index (corresponding to the angle), a range, and a corresponding cartesian coordinate.
    """

    index: int
    range: float
    cartesian: CartesianPoint

    @property
    def position_x(self):
        """
        Retrieve the x-coordinate.
        """
        return self.cartesian.position_x

    @property
    def position_y(self):
        """
        Retrieve the y-coordinate.
        """
        return self.cartesian.position_y

    @property
    def x(self):
        """
        Alternative naming for retrieving the x-coordinate.
        """
        # pylint: disable=invalid-name
        return self.position_x

    @property
    def y(self):
        """
        Alternative naming for retrieving the y-coordinate.
        """
        # pylint: disable=invalid-name
        return self.position_y
