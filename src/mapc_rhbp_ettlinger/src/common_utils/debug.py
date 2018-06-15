import rospy

from agent_knowledge.assemble_task import AssembleKnowledgebase
from agent_knowledge.item import StockItemKnowledgebase
from agent_knowledge.resource import ResourceKnowledgebase


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

        rospy.logerr("------------------------------ Preconditions: ------------------------------------")
        rospy.logerr("active: %s", str(behaviour._active))
        rospy.logerr("isExecuting: %s", str(behaviour._isExecuting))
        for i in range(len(behaviour._preconditions)):
            rospy.logerr("precondition (%s): %s",behaviour._preconditions[i]._name, str(behaviour._get_satisfactions()[i]))

        rospy.logerr("----------------------------------------------------------------------------------")



