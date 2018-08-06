import subprocess
import time

from mac_ros_bridge.msg import Team

import rospy

step = 0
massium = 0

class Stuffaljadlwidj(object):
    def callback_team(self, team):
        self.massium = team.massium
        rospy.logerr(str(team))

    def __init__(self):

        self.massium = 0

        self.start_massim()
        self.start_mapc()

        rospy.Subscriber("/team", Team, self.callback_team)

    def start_mapc(self):
        process_ros = subprocess.Popen(["roslaunch", "mapc_rhbp_ettlinger", "start_med_dev.launch"])

    def start_massim(self):
        process_massim = subprocess.Popen(["sh", "/home/etti/mapc/mapc_workspace/script/med.sh"],
                                          cwd="/home/etti/mapc/massim/massim18/server")


if __name__ == '__main__':

    agent_name = None

    # rospy.init_node('agent_node', anonymous=True, log_level=rospy.ERROR)
    rhbp_agent = Stuffaljadlwidj()
    # rospy.spin()