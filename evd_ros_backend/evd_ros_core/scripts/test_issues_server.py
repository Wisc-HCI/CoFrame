#!/usr/bin/env python

import rospy

from evd_interfaces.issue_client_interface import Issue, IssueClientInterface


class TestIssuesServer:

    def __init__(self):
        self._issue_client = IssueClientInterface()

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

        self._issue_client.issue_submit_pub.publish(issue1)

        issue2 = Issue()
        issue2.source = "test_issues_server"
        issue2.id = "2"
        issue2.level = Issue.LEVEL_WARN
        issue2.data = "{}"
        issue2.human_message = "issue 2"

        self._issue_client.issue_submit_pub.publish(issue2)

        # Get all issues (check if both submitted in there)
        issues = self._issue_client.get_issues(False,'',[],False,'')
        self._check_if_submitted_are_in_list(issues,'for all issues',True,True)

        # Get issues with source filter (check if both submitted is in there)
        issues = self._issue_client.get_issues(True,'test_issues_server',[],False,'')
        self._check_if_submitted_are_in_list(issues,'with source filter',True,True)

        # Get issues with source and id filter (check if one submitted is only one)
        issues = self._issue_client.get_issues(True,'test_issues_server',["2"],False,'')
        self._check_if_submitted_are_in_list(issues,'with source filter',False,True)

        # Get issues with level filter (check if one submitted in there)
        issues =self._issue_client.get_issues(False,'',[],True,Issue.LEVEL_NOTE)
        self._check_if_submitted_are_in_list(issues,'with level filter',True,False)

        # Get issues with different source filter
        issues = self._issue_client.get_issues(True,'imaginary',[],True,Issue.LEVEL_NOTE)
        self._check_if_submitted_are_in_list(issues,'different source filter',False,False)

        # Clear issue 2
        self._issue_client.clear_issue('test_issues_server','2')

        # Check if in server
        issues = self._issue_client.get_issues(True,'test_issues_server',[],False,'')
        self._check_if_submitted_are_in_list(issues,'after clearing 2',True,False)

        # Clear issue 1
        self._issue_client.clear_issue('test_issues_server','1')

        # Check if in server
        issues = self._issue_client.get_issues(False,'',[],False,'')
        self._check_if_submitted_are_in_list(issues,'after clearing 1',False,False)

    def _test_pending_jobs(self):
        source = 'test_issues_server'
        ids = ['1','2','3','4']
        human_messages = ['','','','']
        data = ['{}','{}','{}','{}']

        status = self._issue_client.set_pending_jobs(source, ids, human_messages, data)
        if status != True:
            raise Exception('Setting pending jobs failed')

        jobs, status, message = self._issue_client.get_pending_jobs()
        if status != True:
            raise Exception('Getting pending jobs failed - {0}'.format(message))

        print jobs

        for i in range(0,len(ids)):
            status, message = self._issue_client.clear_pending_job(source,ids[0])

            if not status:
                raise Exception('Failed to clear pending job - {0} - {1}'.format(i,message))

        jobs, status, message = self._issue_client.get_pending_jobs()
        if status != True:
            raise Exception('Getting pending jobs failed - {0}'.format(message))

        print jobs

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
