
import json
import rospy

from evd_ros_core.msg import Issue
from evd_ros_core.srv import GetIssues, GetIssuesRequest, GetIssuesResponse
from evd_ros_core.srv import ClearIssue, ClearIssueRequest, ClearIssueResponse
from evd_ros_core.srv import GetPendingJobs, GetPendingJobsRequest, GetPendingJobsResponse
from evd_ros_core.srv import SetPendingJobs, SetPendingJobsRequest, SetPendingJobsResponse
from evd_ros_core.srv import ClearPendingJob, ClearPendingJobRequest, ClearPendingJobResponse


class IssueClientInterface(object):

    def __init__(self):
        self._cached_issues = None
        self._cached_pending_jobs = None

        self.issue_submit_pub = rospy.Publisher('issue_server/issue_submit',Issue,queue_size=10)

        self.get_issues_srv = rospy.ServiceProxy('issue_server/get_issues',GetIssues)
        self.clear_issue_srv = rospy.ServiceProxy('issue_server/clear_issue',ClearIssue)
        self.get_pending_jobs_srv = rospy.ServiceProxy('issue_server/get_pending_jobs',GetPendingJobs)
        self.set_pending_jobs_srv = rospy.ServiceProxy('issue_server/set_pending_jobs',SetPendingJobs)
        self.clear_pending_job_srv = rospy.ServiceProxy('issue_server/clear_pending_job',ClearPendingJob)

    def submit_issue(self, source, id, level, data, message):
        msg = self.pack_issue_msg(source,id,level,data,message)
        self.issue_submit_pub.publish(msg)

    def pack_issue_msg(self, source, id, level, data, message):
        msg = Issue()
        msg.source = source
        msg.id = msg.id
        msg.level = level
        msg.data = data
        msg.message = message
        return msg

    @property
    def cached_issues(self):
        return self._cached_issues

    @property
    def cached_pending_jobs(self):
        return self._cached_pending_jobs

    def get_issues(self, filter_by_source=False, source='', ids=[], filter_by_level=False, level=''):
        response = self.get_issues_srv(filter_by_source, source, ids, filter_by_level, level)
        self._cached_issues = response.issues
        return self._cached_issues

    def clear_issue(self, source, id):
        response = self.clear_issue_srv(source, id)
        return response.status, response.message

    def get_pending_jobs(self):
        response = self.get_pending_jobs_srv()
        jobs = {}
        for i in range(0,len(response.sources)):
            src = response.sources[i]
            jobs[src] = []

            for j in range(0,len(response.ids[i].data)):
                jobs[src].append({
                    'source': src,
                    'id': response.id[i].data[j],
                    'human_message': respnse.human_message[i].data[j],
                    'data': json.loads(response.data[i].data[j])
                })

        self._cached_pending_jobs = jobs
        return self._cached_pending_jobs, response.status, response.message

    def clear_pending_job(self, source, id):
        response = self.clear_pending_job_srv(source,id)
        return response.status, response.message

    def set_pending_jobs(self, source='', ids=[], human_messages=[], data=[]):

        if len(ids) != len(human_messages) || len(ids) != len(data):
            raise Exception('Lengths must match for all list parameters')

        serializedData = [json.dumps(d) for d in data]
        response = self.set_pending_jobs_srv(source, ids, human_messages, serializedData)
        return response.status
