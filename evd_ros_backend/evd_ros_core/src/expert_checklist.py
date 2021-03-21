#!/usr/bin/env python

'''
Expert Checklist generates all relevant questions / checks for an operator.
'''

import rospy

from std_msgs.msg import Bool
from evd_ros_core.msg import ExpertCheck

from evd_interfaces.data_client_interface import DataClientInterface
from evd_interfaces.issue_client_interface import IssueClientInterface

from evd_ros_core.srv import GetExpertChecklist, GetExpertChecklistRequest, GetExpertChecklistResponse


class ExpertChecklistNode:

    def __init__(self):

        self._checklist = {}

        self._updated_pub = rospy.Publisher('expert_checklist/updated', Bool, queue_size=10)
        self._get_checklist_srv = rospy.Service('expert_checklist/get', GetExpertChecklist, self._get_checklist_cb)

        self._data_client = DataClientInterface(on_program_update_cb=self._program_updated)
        self._issue_client = IssueClientInterface(self._pending_jobs_updated_cb, self._issues_updated_cb)

    def _program_updated(self):
        #
        pass

    def _pending_jobs_updated_cb(self):
        pass

    def _issues_updated_cb(self):
        pass

    def _get_checklist_cb(self, request):
        response = GetExpertChecklistResponse()
        response.list = [self._checklist[key] for key in self._checklist.keys()]
        return response


if __name__ == "__main__":
    rospy.init_node('expert_checklist')

    node = ExpertChecklistNode()
    rospy.spin()
