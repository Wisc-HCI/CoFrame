#!/usr/bin/env python

'''
Provide pending jobs and issues in the EvD system.
'''

#TODO

import rospy

from std_msgs.mgs import Bool
from evd_ros_core.msg import Issue, StringArray

from evd_ros_core.srv import GetIssues, GetIssuesRequest, GetIssuesResponse
from evd_ros_core.srv import ClearIssue, ClearIssueRequest, ClearIssueResponse
from evd_ros_core.srv import GetPendingJobs, GetPendingJobsRequest, GetPendingJobsResponse
from evd_ros_core.srv import SetPendingJobs, SetPendingJobsRequest, SetPendingJobsResponse
from evd_ros_core.srv import ClearPendingJob, ClearPendingJobRequest, ClearPendingJobResponse


class IssueServer:

    def __init__(self):
        self.full_table = {}
        self.pending_jobs = {}

        self._updated_issues_pub = rospy.Publisher('issue_server/updated_issues',Bool, queue_size=10)
        self._updated_pending_pub = rospy.Publisher('issue_server/updated_pending_jobs',Bool,queue_size=10)
        self._issue_sub = rospy.Subscriber('issue_server/issue_submit',Issue,self._issue_submit_cb)

        self._get_issue_srv = rospy.Service('issue_server/get_issues',GetIssues,self._get_issues_cb)
        self._clear_issue_srv = rospy.Service('issue_server/clear_issue',ClearIssue,self._clear_issue_cb)
        self._get_pending_jobs_srv = rospy.Service('issue_server/get_pending_jobs',GetPendingJobs,self._get_pending_jobs_cb)
        self._set_pending_jobs_srv = rospy.Service('issue_server/set_pending_jobs',SetPendingJobs,self._set_pending_jobs_cb)
        self._clear_pending_job_srv = rospy.Service('issue_server/clear_pending_job',ClearPendingJob,self._clear_pending_job_cb)

    def _issue_submit_cb(self, msg):

        if msg.source not in self.full_table.keys():
            self.full_table[msg.source] = {}

        self.full_table[msg.source][msg.id] = msg
        self._updated_issues_pub.publish(Bool(True))

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
                    src_filter_list = self.full_table[request.source].values()
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

                self._updated_issues_pub.publish(Bool(True))
            else:
                response.message = 'ID not in table'
        else:
            response.message = 'Source not in table'

        return response

    def _get_pending_jobs_cb(self, request):
        response = GetPendingJobsResponse()

        for source in self.pending_jobs.keys():

            ids = StringArray()
            msgs = StringArray()
            data = StringArray()
            for id in self.pending_jobs[source].keys():
                dct = self.pending_jobs[source][id]
                ids.data.append(id)
                msgs.data.append(dct['human_message'])
                data.data.append(dct['data'])

            response.sources.append(source)
            response.ids.append(ids)
            response.human_messages.append(msgs)
            response.data.append(data)

        response.message = ''
        response.status = True
        return response

    def _set_pending_jobs_cb(self, request):

        if request.source not in self.pending_jobs.keys():
            self.pending_jobs[request.source] = {}

        for i in range(0,len(request.ids)):
            self.pending_jobs[request.source][request.ids[i]] = {
                'human_message': request.human_messages[i],
                'data': request.data[i]
            }

        self._updated_pending_pub.publish(Bool(True))

        response = SetPendingJobsResponse()
        response.status = True
        return response

    def _clear_pending_job_cb(self, request):
        response = ClearPendingJobResponse()
        response.status = False

        if request.source in self.pending_jobs.keys():
            if request.id in self.pending_jobs[request.source].keys():
                response.status = True
                response.message = ''

                self._updated_pending_pub.publish(Bool(True))
            else:
                response.message = 'ID not in table'
        else:
            response.message = 'Source not in table'

        return response


if __name__ == "__main__":
    rospy.init_node('issue_server')

    node = IssueServer()
    rospy.spin()
