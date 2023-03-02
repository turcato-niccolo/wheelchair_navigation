import math

import numpy as np


class PolarNav:
    """
        Polar coordinates navigation
    """

    def update(self, map):
        self.map = map
        return

    def get_map(self):
        return self.map

    def get_heading(self):
        raise NotImplementedError


class PolarPotentialFieldNav(PolarNav):
    def __int__(self, num_cells, num_sec, max_radius, max_angle, goal_weight=-10.0, obstacle_weight=10.0):
        """
        :param num_cells:    Number of cells for each polar sector
        :param num_sec:    Number of sectors, should be odd, the middle sector is the forward direction
        :return:
        :rtype:
        """
        self.num_cells = num_cells
        self.num_sec = num_sec
        self.max_angle = max_angle
        self.max_radius = max_radius
        self.goal_weight = goal_weight
        self.obstacle_weight = obstacle_weight

        self.map = np.zeros((num_cells, num_sec), float)
        self.angles = np.arange(-self.max_angle, self.max_angle, int(2*max_angle/num_sec))
        return

    def get_heading(self):
        """"""
        grad = self.map[-1, :] - self.map[:-1, :]
        return self.angles[np.argmin(np.mean(grad, axis=0))]
        #return int((self.num_sec-1)/2)

    def get_polar_indices(self, r, theta):
        sector_angle = 2 * self.max_angle / self.num_sec
        cell_len = self.max_radius / self.num_cells
        sector = self.num_sec - 1 - math.floor((self.max_angle + theta) / sector_angle)
        cell = self.num_cells - 1 - math.floor(r / cell_len)
        return cell, sector
    def set_goal(self, r, theta):
        cell, sector = self.get_polar_indices(r, theta)

        for i in range(self.num_cells):
            for j in range(self.num_sec):
                self.map[i, j] += self.goal_weight * np.exp(-((sector-j)**2 + (cell-i)**2) / np.abs(self.goal_weight))

    def add_obstacle(self, r, theta):
        cell, sector = self.get_polar_indices(r, theta)

        for i in range(self.num_cells):
            for j in range(self.num_sec):
                self.map[i, j] += self.obstacle_weight * np.exp(-((sector - j) ** 2 + (cell - i) ** 2) / np.abs(self.obstacle_weight))
