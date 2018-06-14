from __builtin__ import xrange

import rospy
from mapc_rhbp_ettlinger.msg import Task
from mac_ros_bridge.msg import RequestAction, Job, SimStart

from common_utils.agent_utils import AgentUtils
from agent_knowledge.tasks import JobKnowledgebase


class JobPlanner(object):

    def __init__(self, agent_name):

        rospy.init_node('planner_node', anonymous=True, log_level=rospy.ERROR)

        self.all_jobs = []
        self.all_tasks = []

        self._task_knowledge = JobKnowledgebase()

        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=agent_name)

        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)
        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)


    def _action_request_callback(self, requestAction):
        """
        here we just trigger the decision-making and plannig
        :param msg: The request for action message
        :type msg: RequestAction
        :return:
        """
        # get all jobs from request
        all_jobs_new = self.extract_jobs(requestAction)

        for job in all_jobs_new:
            # if job has not been seen before -> process it
            if job not in self.all_jobs:
                rospy.loginfo("JobPlanner:: processing new job %s", job.id)
                tasks = self.extract_tasks_from_job(job)
                for task in tasks:
                    rospy.loginfo("JobPlanner::              new task: %s -> %s", task.item, job.storage_name)
                self.all_tasks += tasks

        self.all_jobs = all_jobs_new


        rospy.loginfo("JobPlanner:: Jobs processed")

    def extract_jobs(self, msg):
        """
        Extracts all jobs from the RequestAction into a list
        :param msg: The request for action message
        :type msg: RequestAction
        :return:
        """

        extracted_jobs = []

        for mission in msg.mission_jobs:
            extracted_jobs += [mission]
        #for auction in msg.auction_jobs:
        #    all_jobs_new.add(auction.job)
        for priced in msg.priced_jobs:
            extracted_jobs += [priced]

        return extracted_jobs


    def extract_tasks_from_job(self, job):
        """
        Divides the job in small tasks that can be performed together by multiple agents
        :param      job         : the job to decompose
        :type       job         : Job
        :return:    Task[]
        """
        tasks = []
        id = 0
        for item in job.items:
            for i in xrange(item.amount):
                task = Task(
                    job_id=job.id,
                    id=str(id),
                    destination=job.storage_name,
                    item=item.name,
                )
                tasks.append(task)
                self._task_knowledge.save_task(task)
                id = id+1
        return tasks

    def _sim_start_callback(self, msg):
        self._task_knowledge._callback_sim_start(msg)



if __name__ == '__main__':

    try:

        # Just take a random agent. doesn't really matter
        job_planner = JobPlanner(
            agent_name="agentA1")

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
