from __builtin__ import xrange

import rospy
from mac_rhbp_example.msg import Task
from mac_ros_bridge.msg import RequestAction, Job

from sets import Set

from agent_knowledge.tasks import TaskKnowledge


class JobPlanner:

    # TODO: Auction jobs
    def __init__(self, agent_name):
        self.current_step = 0
        self.all_jobs = [] # Temp variable for testing
        self.all_tasks = [] # Temp variable for testing

        self._task_knowledge = TaskKnowledge(agent_name=agent_name)

        #rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)
        # For now passing it from agent


    def _action_request_callback(self, requestAction):
        """
        here we just trigger the decision-making and plannig
        :param msg: the message
        :type msg: RequestAction
        :return:
        """

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

        extracted_jobs = []

        for mission in msg.mission_jobs:
            extracted_jobs += [mission]
        #for auction in msg.auction_jobs:
        #    all_jobs_new.add(auction.job)
        for priced in msg.priced_jobs:
            extracted_jobs += [priced]

        return extracted_jobs


    def discard_old_jobs(self, deleted_jobs):
        deleted_job_ids = [job.id for job in deleted_jobs]

        self.all_tasks = [task for task in self.all_tasks if task.job_id not in deleted_job_ids]


    def extract_tasks_from_job(self, job):
        """
        :param job : the job to decompose
        :type job  : Job
        :return:Task
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
