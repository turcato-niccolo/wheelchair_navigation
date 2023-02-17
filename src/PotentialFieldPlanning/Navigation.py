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
    def __int__(self, num_cells, num_sec):
        """
        :param num_cells:    Number of cells for each polar sector
        :param num_sec:    Number of sectors, should be odd, the middle sector is the forward direction
        :return:
        :rtype:
        """
        self.num_cells = num_cells
        self.num_sec = num_sec

        self.map = np.zeros((num_cells, num_sec), float)
        return

    def get_heading(self):
        """"""
        grad = self.map[-1, :] - self.map[:-1, :]
        return np.argmin(np.mean(grad, axis=0))
        #return int((self.num_sec-1)/2)
