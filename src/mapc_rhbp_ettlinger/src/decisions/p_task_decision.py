from so_data.patterns import DecisionPattern


class CurrentTaskDecision(DecisionPattern):

    TYPE_ASSEMBLE = "assemble"
    TYPE_DELIVER = "deliver"
    TYPE_BUILD_WELL = "build_well"

    def __init__(self, agent_name, task_type):

        self.agent_name = agent_name
        self.task_type = task_type

        super(CurrentTaskDecision, self).__init__(buffer=None, frame=None, requres_pos=False)
        self.current_task = None

    def calc_value(self):
        return [self.current_task, self.state]

    def start_task(self, task):
        self.current_task = task

    def end_task(self):
        self.current_task = None