from agent_knowledge.resource import ResourceKnowledgebase


class DebugUtils:

    def instant_find_resources(self, resource_knowledgebase):
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




