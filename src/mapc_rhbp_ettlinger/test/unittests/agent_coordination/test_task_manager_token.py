"""
@author: krakowczyk
"""

PKG="group1_ws17"
import roslib; roslib.load_manifest(PKG)

import unittest

from agent_coordination.task_manager import is_task_manager

class TaskManagerTokenTestSuite(unittest.TestCase):

    def test_team_size_zero(self):
        self.assertFalse(is_task_manager(1,0,0))
        self.assertFalse(is_task_manager(1,0,1))

    def test_team_size_one(self):
        self.assertTrue(is_task_manager(1,1,0))
        self.assertTrue(is_task_manager(1,1,1))

    def test_team_size_two_step_zero(self):
        self.assertTrue(is_task_manager(1,2,0))
        self.assertFalse(is_task_manager(2,2,0))

    def test_team_size_two_step_one(self):
        self.assertFalse(is_task_manager(1,2,1))
        self.assertTrue(is_task_manager(2,2,1))

    def test_team_size_two_step_two(self):
        self.assertTrue(is_task_manager(1,2,2))
        self.assertFalse(is_task_manager(2,2,2))

    def test_team_size_two_step_three(self):
        self.assertFalse(is_task_manager(1,2,3))
        self.assertTrue(is_task_manager(2,2,3))


if __name__ == '__main__':
    #unittest.main()
    import rosunit
    rosunit.unitrun(PKG, 'test_task_manager_token',
                    TaskManagerTokenTestSuite)
