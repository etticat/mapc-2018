import time

from mac_ros_bridge.msg import Position
from mapc_rhbp_ettlinger.msg import Task

import rospy
from agent_knowledge.item import StockItemKnowledgeBase
from agent_knowledge.task import TaskKnowledgeBase
from common_utils import etti_logging

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.utils.debug')

class DebugUtils:

    FILE_HANDLER_TIME = None
    FILE_HANDLER_STEP = None

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
        stock_item_knowledgebase = StockItemKnowledgeBase()
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
    def add_build_well_task(agent_name="agentA1"):

        ettilog.logerr("------------------------------ Building well: ------------------------------------")
        well_task_knowledebase = TaskKnowledgeBase()
        well_task_knowledebase.create_task(Task(
            agent_name = agent_name,
            pos = Position(lat=0.0, long=0.0),
            task = "wellType2",
            type = TaskKnowledgeBase.TYPE_BUILD_WELL
        ))

        ettilog.logerr("----------------------------------------------------------------------------------")

    @staticmethod
    def assign_assembly_task():

        assembly_knowledgebase= TaskKnowledgeBase()

        assembly_knowledgebase.create_task(Task(
            id = 'edawd',
            agent_name = 'agentA1',
            pos = Position(long=2.31017, lat=48.82456),
            task = 'assemble:item8',
            type = TaskKnowledgeBase.TYPE_ASSEMBLE
        ))


    @staticmethod
    def start_thread_counter():
        import threading
        t = threading.Thread(target=DebugUtils.write_each_sec)
        t.start()
        pass

    @staticmethod
    def write_each_sec():
        i = 0
        while True:
            i = i + 1
            import threading

            import os
            import psutil
            process = psutil.Process(os.getpid())
            ram = process.memory_info().rss

            all_threads = threading.Thread.count_ttt
            count = threading.activeCount()
            rospy.logerr(str(count) + "," + str(i) + "," + str(all_threads) + "   Ram: " + str(ram / 1024 / 1024 ))
            time.sleep(1.0)
