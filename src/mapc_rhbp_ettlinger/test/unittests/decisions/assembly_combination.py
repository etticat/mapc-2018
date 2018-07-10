"""
@author: krakowczyk
"""
from decisions.assembly_combination import AssemblyCombinationDecision

PKG="group1_ws17"
import roslib; roslib.load_manifest(PKG)

import unittest

from agent_utils.name_helpers import get_agent_id, get_agent_team

class NameHelpersTestSuite(unittest.TestCase):

    def test_team_agentA1(self):
        choose_best_assembly_combination = AssemblyCombinationDecision()

        finished_products = {}

        roles = ["car", "motorcycle", "drone", "truck"]

        finished_products["item5"] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        finished_products["item5"][1] = 1
        finished_products["item5"][4] = 1
        finished_products["item5"][5] = -1
        finished_products["item5"][11 + roles.index("drone")] = 1
        finished_products["item5"][11 + roles.index("car")] = 1

        finished_products["item6"] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        finished_products["item6"][0] = 1
        finished_products["item6"][1] = 1
        finished_products["item6"][2] = 1
        finished_products["item6"][3] = 1
        finished_products["item6"][4] = 1
        finished_products["item6"][6] = -1
        finished_products["item6"][11 + roles.index("motorcycle")] = 1
        finished_products["item6"][11 + roles.index("truck")] = 1

        finished_products["item7"] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        finished_products["item7"][0] = 1
        finished_products["item7"][1] = 1
        finished_products["item7"][2] = 1
        finished_products["item7"][3] = 1
        finished_products["item7"][4] = 1
        finished_products["item7"][7] = -1
        finished_products["item7"][11 + roles.index("motorcycle")] = 1
        finished_products["item7"][11 + roles.index("car")] = 1

        finished_products["item8"] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        finished_products["item8"][0] = 1
        finished_products["item8"][4] = 1
        finished_products["item8"][8] = -1
        finished_products["item8"][11 + roles.index("drone")] = 1
        finished_products["item8"][11 + roles.index("car")] = 1

        finished_products["item9"] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        finished_products["item9"][0] = 1
        finished_products["item9"][1] = 1
        finished_products["item9"][4] = 1
        finished_products["item9"][6] = 1
        finished_products["item9"][7] = 1
        finished_products["item9"][9] = -1
        finished_products["item9"][11 + roles.index("motorcycle")] = 1
        finished_products["item9"][11 + roles.index("truck")] = 1

        finished_products["item10"] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        finished_products["item10"][1] = 1
        finished_products["item10"][5] = 1
        finished_products["item10"][6] = 1
        finished_products["item10"][10] = -1
        finished_products["item10"][11 + roles.index("motorcycle")] = 1
        finished_products["item10"][11 + roles.index("car")] = 1

        import numpy as np

        stock = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        stock[0] = 3
        stock[1] = 3
        stock[2] = 2
        stock[3] = 2
        stock[4] = 3
        stock[11 + roles.index("motorcycle")] = 999
        stock[11 + roles.index("car")] = 999
        stock[11 + roles.index("drone")] = 999
        stock[11 + roles.index("truck")] = 999

        priorities = {}
        priorities["item0"] = 0.0
        priorities["item1"] = 0.0
        priorities["item2"] = 0.0
        priorities["item3"] = 0.0
        priorities["item4"] = 0.0
        priorities["item5"] = 0.1
        priorities["item6"] = 0.1
        priorities["item7"] = 0.1
        priorities["item8"] = 0.1
        priorities["item9"] = 0.1
        priorities["item10"] = 0.1

        finished_item_list = finished_products.keys()
        finished_item_list.sort()

        WEIGHT_PRODUCT_COUNT = -1
        WEIGHT_PRIORITY = 10

        for i in finished_products.keys():
            finished_products[i] = np.array(finished_products[i])

        value, items = choose_best_assembly_combination.try_build_item(stock, priorities)

        assert value == 0.0
        assert "item6" in items
        assert "item7" in items
        assert "item9" in items
        assert len(items) == 3


if __name__ == '__main__':
    #unittest.main()
    import rosunit
    rosunit.unitrun(PKG, 'test_name_helpers',
                    NameHelpersTestSuite)
