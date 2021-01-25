#!/usr/bin/env python

import rospy

from evd_ros_core.msg import Issue
from evd_ros_core.srv import GetIssues, GetIssuesRequest, GetIssuesResponse
from evd_ros_core.srv import ClearIssue, ClearIssueRequest, ClearIssueResponse
from evd_ros_core.srv import GetPendingJobs, GetPendingJobsRequest, GetPendingJobsResponse
from evd_ros_core.srv import SetPendingJobs, SetPendingJobsRequest, SetPendingJobsResponse
from evd_ros_core.srv import ClearPendingJob, ClearPendingJobRequest, ClearPendingJobResponse


class TestIssuesServer:

    def __init__(self):
        self._issue_submit_pub = rospy.Publisher('issue_server/issue_submit',Issue, queue_size=5)
        self._get_issues_srv = rospy.ServiceProxy('issue_server/get_issues',GetIssues)
        self._clear_issue_srv = rospy.ServiceProxy('issue_server/clear_issue',ClearIssue)
        self._get_pending_jobs_srv = rospy.ServiceProxy('issue_server/get_pending_jobs',GetPendingJobs)
        self._set_pending_jobs_srv = rospy.ServiceProxy('issue_server/set_pending_jobs',SetPendingJobs)
        self._clear_pending_job_srv = rospy.ServiceProxy('issue_server/clear_pending_job',ClearPendingJob)

    def spin(self):

        # Give ROS some setup time
        rospy.sleep(5)

        self._test_issues()

        self._test_pending_jobs()

        # done, let node rest
        rospy.spin()

    def _test_issues(self):
        # Sumbit a couple of issues
        issue1 = Issue()
        issue1.source = "test_issues_server"
        issue1.id = "1"
        issue1.level = Issue.LEVEL_NOTE
        issue1.data = "{}"
        issue1.human_message = "issue 1"

        self._issue_submit_pub.publish(issue1)

        issue2 = Issue()
        issue2.source = "test_issues_server"
        issue2.id = "2"
        issue2.level = Issue.LEVEL_WARN
        issue2.data = "{}"
        issue2.human_message = "issue 2"

        self._issue_submit_pub.publish(issue2)

        # Get all issues (check if both submitted in there)
        issues = self._get_issues_srv(False,'',[],False,'').issues
        self._check_if_submitted_are_in_list(issues,'for all issues',True,True)

        # Get issues with source filter (check if both submitted is in there)
        issues = self._get_issues_srv(True,'test_issues_server',[],False,'').issues
        self._check_if_submitted_are_in_list(issues,'with source filter',True,True)

        # Get issues with source and id filter (check if one submitted is only one)
        issues = self._get_issues_srv(True,'test_issues_server',["2"],False,'').issues
        self._check_if_submitted_are_in_list(issues,'with source filter',False,True)

        # Get issues with level filter (check if one submitted in there)
        issues = self._get_issues_srv(False,'',[],True,Issue.LEVEL_NOTE).issues
        self._check_if_submitted_are_in_list(issues,'with level filter',True,False)

        # Get issues with different source filter
        issues = self._get_issues_srv(True,'imaginary',[],True,Issue.LEVEL_NOTE).issues
        self._check_if_submitted_are_in_list(issues,'different source filter',False,False)

        # Clear issue 2
        self._clear_issue_srv('test_issues_server','2')

        # Check if in server
        issues = self._get_issues_srv(True,'test_issues_server',[],False,'').issues
        self._check_if_submitted_are_in_list(issues,'after clearing 2',True,False)

        # Clear issue 1
        self._clear_issue_srv('test_issues_server','1')

        # Check if in server
        issues = self._get_issues_srv(False,'',[],False,'').issues
        self._check_if_submitted_are_in_list(issues,'after clearing 1',False,False)

    def _test_pending_jobs(self):
        source = 'test_issues_server'
        ids = ['1','2','3','4']
        human_messages = ['','','','']
        data = ['{}','{}','{}','{}']

        resp = self._set_pending_jobs_srv(source, ids, human_messages, data)
        if resp.status != True:
            raise Exception('Setting pending jobs failed')

        resp = self._get_pending_jobs_srv()
        if resp.status != True:
            raise Exception('Getting pending jobs failed - {0}'.format(resp.message))

        print resp

        for i in range(0,len(ids)):
            resp = self._clear_pending_job_srv(source,ids[0])

            if not resp.status:
                raise Exception('Failed to clear pending job - {0} - {1}'.format(i,resp.message))

        resp = self._get_pending_jobs_srv()
        if resp.status != True:
            raise Exception('Getting pending jobs failed - {0}'.format(resp.message))

        print resp

    def _check_if_submitted_are_in_list(self, issues, condition_str='', should_find_1=True, should_find_2=True):

        found1 = False
        found2 = False
        for i in issues:
            if i.source == 'test_issues_server':
                if i.id == "1":
                    found1 = True
                elif i.id == "2":
                    found2 = True

        if should_find_1 and not found1:
            raise Exception("Issue 1 was missing when checking {}".format(condition_str))
        elif not should_find_1 and found1:
            raise Exception("Issue 1 was found but should not be when checking {}".format(condition_str))

        if should_find_2 and not found2:
            raise Exception("Issue 2 was missing when checking {}".format(condition_str))
        elif not should_find_2 and found2:
            raise Exception("Issue 2 was found but should not be when checking {}".format(condition_str))


if __name__ == "__main__":
    rospy.init_node('test_issues_server')

    node = TestIssuesServer()
    node.spin()
