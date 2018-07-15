import time

from mac_ros_bridge.msg import ResourceMsg, Resource

import rospy
from common_utils import etti_logging
from provider.facility_provider import FacilityProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.utils.debug')


class DebugUtils:
    """
    Debug util providing some methods to help while development
    """
    FILE_HANDLER_TIME = None
    FILE_HANDLER_STEP = None

    @staticmethod
    def instant_find_resources():
        """
        Instantly finds all resources in the beginning. Only works for the default seed ad configuration
        :return:
        """

        rsMsg = ResourceMsg()

        tuples = [("item0", "node0", 48.8222885132, 2.28051996231),
                  ("item1", "node2", 48.8290290833, 2.28021001816),
                  ("item1", "node5", 48.8563308716, 2.29516005516),
                  ("item2", "node1", 48.8847084045, 2.28618001938),
                  ("item3", "node6", 48.8477783203, 2.31370997429),
                  ("item4", "node10", 48.8651008606, 2.34387993813),
                  ("item4", "node3", 48.8625602722, 2.32475996017)]
        for tuple in tuples:
            r = Resource()
            r.pos.lat = tuple[2]
            r.pos.long = tuple[3]
            r.name = tuple[1]
            r.item.name = tuple[0]
            rsMsg.facilities.append(r)

        facility_provider = FacilityProvider()
        facility_provider.resources_callback(rsMsg)

    @staticmethod
    def print_precondition_states(behaviour):
        """
        Prints information about the preconditions of a behaviour
        :param behaviour:
        :return:
        """

        ettilog.logerr("------------------------------ Preconditions: ------------------------------------")
        ettilog.logerr("active: %s", str(behaviour._active))
        ettilog.logerr("isExecuting: %s", str(behaviour._isExecuting))
        for i in range(len(behaviour._preconditions)):
            ettilog.logerr("precondition (%s): %s", behaviour._preconditions[i]._name,
                           str(behaviour._get_satisfactions()[i]))

        ettilog.logerr("----------------------------------------------------------------------------------")
