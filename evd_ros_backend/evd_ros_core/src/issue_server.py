#!/usr/bin/env python

'''
'''

import rospy

from evd_ros_core.msg import Issue
from evd_ros_core.srv import GetIssues, GetIssuesRequest, GetIssuesResponse
from evd_ros_core.srv import ClearIssue, ClearIssueRequest, ClearIssueResponse


class IssueServer:

    def __init__(self):
        self.full_table = {}

        self._issue_sub = rospy.Subscriber('issue_server/issue_submit',Issue,self._issue_submit_cb)
        self._get_issue_srv = rospy.Service('issue_server/get_issues',GetIssues,self._get_issues_cb)
        self._clear_issue_srv = rospy.Service('issue_server/clear_issue',ClearIssue,self._clear_issue_cb)

    def _issue_submit_cb(self, msg):

        if self.source not in self.full_table.keys():
            self.full_table[msg.source] = {}

        self.full_table[msg.source][msg.id] = msg

    def _get_issues_cb(self, request):
        response = GetIssuesResponse()

        src_filter_list = []
        if request.filter_by_source:
            if request.source in self.full_table:
                if len(request.ids) > 0:
                    for id in request.ids:
                        if id in self.full_table[request.source]:
                            src_filter_list.append(self.full_table[request.source][id])
                else:
                    src_filter_list = self.full_table[reuest.source].values()
        else:
            for source in self.full_table.keys():
                for id in self.full_table[source].keys():
                    src_filter_list.append(self.full_table[source][id])

        lvl_filter_list = []
        if request.filter_by_level:
            for issue in src_filter_list:
                if issue.level == request.level:
                    lvl_filter_list.append(issue)
        else:
            lvl_filter_list = src_filter_list

        response.issues = lvl_filter_list
        return response

    def _clear_issue_cb(self, request):
        response = ClearIssueResponse()
        response.status = False

        if request.source in self.full_table.keys():
            if request.id in self.full_table[request.source].keys():
                response.status = True
                response.message = ''
                self.full_table[request.source].pop(request.id)
            else:
                response.message = 'ID not in table'
        else:
            response.message = 'Source not in table'

        return response


if __name__ == "__main__":
    rospy.init_node('issue_server')

    node = IssueServer()
    rospy.spin()
