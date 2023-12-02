import numpy as np
from enum import Enum
from scipy.spatial.transform import Rotation as R


class LShapeFitting:
    """
    LShapeFitting class. You can use this class by initializing the class and
    changing the parameters, and then calling the fitting method.

    """

    class Criteria(Enum):
        AREA = 1
        CLOSENESS = 2
        VARIANCE = 3

    def __init__(self):
        """
        Default parameter settings
        """
        #: Fitting criteria parameter
        self.criteria = self.Criteria.VARIANCE
        #: Minimum distance for closeness criteria parameter [m]
        self.min_dist_of_closeness_criteria = 0.01
        #: Angle difference parameter [deg]
        self.d_theta_deg_for_search = 1.0
        #: Range segmentation parameter [m]
        self.R0 = 3.0
        #: Range segmentation parameter [m]
        self.Rd = 0.001

    def fitting(self, points):
        """
        Fitting L-shape model to object points

        Parameters
        ----------
        ox : x positions of range points from an object
        oy : y positions of range points from an object

        Returns
        -------
        rect
        """
        return self._rectangle_search(points[:, 0], points[:, 1])

    @staticmethod
    def _calc_area_criterion(c1, c2):
        c1_max, c1_min, c2_max, c2_min = LShapeFitting._find_min_max(c1, c2)
        alpha = -(c1_max - c1_min) * (c2_max - c2_min)
        return alpha

    def _calc_closeness_criterion(self, c1, c2):
        c1_max, c1_min, c2_max, c2_min = LShapeFitting._find_min_max(c1, c2)

        # Vectorization
        d1 = np.minimum(c1_max - c1, c1 - c1_min)
        d2 = np.minimum(c2_max - c2, c2 - c2_min)
        d = np.maximum(np.minimum(d1, d2), self.min_dist_of_closeness_criteria)
        beta = (1.0 / d).sum()

        return beta

    @staticmethod
    def _calc_variance_criterion(c1, c2):
        c1_max, c1_min, c2_max, c2_min = LShapeFitting._find_min_max(c1, c2)

        # Vectorization
        d1 = np.minimum(c1_max - c1, c1 - c1_min)
        d2 = np.minimum(c2_max - c2, c2 - c2_min)
        e1 = d1[d1 < d2]
        e2 = d2[d1 >= d2]
        v1 = - np.var(e1) if len(e1) > 0 else 0.
        v2 = - np.var(e2) if len(e2) > 0 else 0.
        gamma = v1 + v2

        return gamma

    @staticmethod
    def _find_min_max(c1, c2):
        c1_max = max(c1)
        c2_max = max(c2)
        c1_min = min(c1)
        c2_min = min(c2)
        return c1_max, c1_min, c2_max, c2_min

    @staticmethod
    def rot_mat_2d(angle):
        """
        Create 2D rotation matrix from an angle
        """
        return R.from_euler('z', angle).as_matrix()[0:2, 0:2]

    def _rectangle_search(self, x, y):

        xy = np.array([x, y]).T

        d_theta = np.deg2rad(self.d_theta_deg_for_search)
        min_cost = (-float('inf'), None)
        for theta in np.arange(0.0, np.pi / 2.0 - d_theta, d_theta):

            c = xy @ self.rot_mat_2d(theta)
            c1 = c[:, 0]
            c2 = c[:, 1]

            # Select criteria
            cost = 0.0
            if self.criteria == self.Criteria.AREA:
                cost = self._calc_area_criterion(c1, c2)
            elif self.criteria == self.Criteria.CLOSENESS:
                cost = self._calc_closeness_criterion(c1, c2)
            elif self.criteria == self.Criteria.VARIANCE:
                cost = self._calc_variance_criterion(c1, c2)

            if min_cost[0] < cost:
                min_cost = (cost, theta)

        # calc best rectangle
        sin_s = np.sin(min_cost[1])
        cos_s = np.cos(min_cost[1])

        c1_s = xy @ np.array([cos_s, sin_s]).T
        c2_s = xy @ np.array([-sin_s, cos_s]).T

        rect = RectangleData()
        rect.a[0] = cos_s
        rect.b[0] = sin_s
        rect.c[0] = min(c1_s)
        rect.a[1] = -sin_s
        rect.b[1] = cos_s
        rect.c[1] = min(c2_s)
        rect.a[2] = cos_s
        rect.b[2] = sin_s
        rect.c[2] = max(c1_s)
        rect.a[3] = -sin_s
        rect.b[3] = cos_s
        rect.c[3] = max(c2_s)

        r_matrix = [[cos_s, -sin_s, 0],
                    [sin_s, cos_s, 0],
                    [0, 0, 1]]

        rect.quat = R.from_matrix(r_matrix).as_quat()
        rect.calc_rect_contour()
        rect.calc_center_size()
        return rect


class RectangleData:

    def __init__(self):
        self.a = [None] * 4
        self.b = [None] * 4
        self.c = [None] * 4

        self.rect_c_x = [None] * 4
        self.rect_c_y = [None] * 4

        self.quat = [None] * 4
        self.center = [None] * 2
        self.size = [None] * 2

    def calc_rect_contour(self) -> None:

        self.rect_c_x[0], self.rect_c_y[0] = self.calc_cross_point(
            self.a[0:2], self.b[0:2], self.c[0:2])
        self.rect_c_x[1], self.rect_c_y[1] = self.calc_cross_point(
            self.a[1:3], self.b[1:3], self.c[1:3])
        self.rect_c_x[2], self.rect_c_y[2] = self.calc_cross_point(
            self.a[2:4], self.b[2:4], self.c[2:4])
        self.rect_c_x[3], self.rect_c_y[3] = self.calc_cross_point(
            [self.a[3], self.a[0]], [self.b[3], self.b[0]], [self.c[3], self.c[0]])

    def calc_center_size(self) -> None:
        minX = np.min(self.rect_c_x)
        minY = np.min(self.rect_c_y)
        maxX = np.max(self.rect_c_x)
        maxY = np.max(self.rect_c_y)
        self.center = [(maxX + minX)/2.0, (maxY + minY)/2.0]

        p1 = np.array([self.rect_c_x[0], self.rect_c_y[0]])
        p2 = np.array([self.rect_c_x[1], self.rect_c_y[1]])
        p3 = np.array([self.rect_c_x[2], self.rect_c_y[2]])

        side1 = np.linalg.norm(p1-p2)
        side2 = np.linalg.norm(p2-p3)

        yaw = self.euler_from_quaternion(
            self.quat[0], self.quat[1], self.quat[2], self.quat[3])
        size = [0,0]
        if(side1 > side2) or (yaw > 0.785):
            size = [side1, side2]
        else:
            size = [side2, side1]
        self.size = size

    @staticmethod
    def calc_cross_point(a, b, c):
        x = (b[0] * -c[1] - b[1] * -c[0]) / (a[0] * b[1] - a[1] * b[0])
        y = (a[1] * -c[0] - a[0] * -c[1]) / (a[0] * b[1] - a[1] * b[0])
        return x, y
    
    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        yaw is rotation around z in radians (counterclockwise)

        Note: only returns yaw about z axis
        """
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return yaw_z  # in radians
