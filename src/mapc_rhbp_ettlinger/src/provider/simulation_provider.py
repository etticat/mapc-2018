from common_utils.singleton import Singleton


class SimulationProvider(object):
    __metaclass__ = Singleton
    
    
    def __init__(self):
        self.min_lat = 0
        self.max_lat = 0
        self.min_long = 0
        self.max_long = 0

    def callback_sim_start(self, sim_start):
        """

        :param sim_start:
        :type sim_start: sim_start
        :return:
        """
        self.min_lat = sim_start.min_lat
        self.max_lat = sim_start.max_lat
        self.min_long = sim_start.min_lon
        self.max_long = sim_start.max_lon
        pass
