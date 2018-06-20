from mac_ros_bridge.msg import Position
from mapc_rhbp_ettlinger.msg import WellTask

from agent_knowledge.item import StockItemKnowledgebase
from agent_knowledge.well import WellTaskKnowledgebase
from common_utils import rhbp_logging

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.utils.debug')

class DebugUtils:

    @staticmethod
    def instant_find_resources(resource_knowledgebase):
        resource_knowledgebase._kb_client.update(
            ("resource", "item4", "node10", "*", "*"),
            ("resource", "item4", "node10", "48.8651008606", "2.34387993813"),
            push_without_existing = True
        )
        resource_knowledgebase._kb_client.update(
            ("resource", "item4", "node3"),
            ("resource", "item4", "node3", "48.8625602722", "2.32475996017"),
            push_without_existing = True
        )
        resource_knowledgebase._kb_client.update(
            ("resource", "item1", "node5"),
            ("resource", "item1", "node5","48.8563308716","2.29516005516"),
            push_without_existing = True
        )
        resource_knowledgebase._kb_client.update(
            ("resource", "item0", "node0"),
            ("resource", "item0", "node0", "48.8222885132", "2.28051996231"),
            push_without_existing = True
        )
        resource_knowledgebase._kb_client.update(
            ("resource", "item3", "node6"),
            ("resource", "item3", "node6", "48.8477783203", "2.31370997429"),
            push_without_existing = True
        )
        resource_knowledgebase._kb_client.update(
            ("resource", "item1", "node2"),
            ("resource", "item1", "node2", "48.8290290833", "2.28021001816"),
            push_without_existing = True
        )

        resource_knowledgebase._kb_client.update(
            ("resource", "item2", "node1"),
            ("resource", "item2", "node1", "48.8847084045", "2.28618001938"),
            push_without_existing = True
        )


    @staticmethod
    def show_total_stock_with_goals():
        stock_item_knowledgebase = StockItemKnowledgebase()
        return stock_item_knowledgebase.get_total_stock_and_goals()

    @staticmethod
    def print_precondition_states(behaviour):

        ettilog.logerr("------------------------------ Preconditions: ------------------------------------")
        ettilog.logerr("active: %s", str(behaviour._active))
        ettilog.logerr("isExecuting: %s", str(behaviour._isExecuting))
        for i in range(len(behaviour._preconditions)):
            ettilog.logerr("precondition (%s): %s",behaviour._preconditions[i]._name, str(behaviour._get_satisfactions()[i]))

        ettilog.logerr("----------------------------------------------------------------------------------")



    @staticmethod
    def add_build_well_task():

        ettilog.logerr("------------------------------ Building well: ------------------------------------")
        well_task_knowledebase = WellTaskKnowledgebase()
        well_task_knowledebase.save_task(WellTask(
            agent_name = 'agentA1',
            pos = Position(lat="48.84778", long="2.33733"),
            well_type = "wellType1",
            built=False
        ))

        ettilog.logerr("----------------------------------------------------------------------------------")



