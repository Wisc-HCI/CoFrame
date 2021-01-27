
import rospy

from evd_ros_core.msg import Issue
from evd_ros_core.srv import GetIssues, GetIssuesRequest, GetIssuesResponse
from evd_ros_core.srv import ClearIssue, ClearIssueRequest, ClearIssueResponse
from evd_ros_core.srv import GetPendingJobs, GetPendingJobsRequest, GetPendingJobsResponse
from evd_ros_core.srv import SetPendingJobs, SetPendingJobsRequest, SetPendingJobsResponse
from evd_ros_core.srv import ClearPendingJob, ClearPendingJobRequest, ClearPendingJobResponse


class IssueClientInterface:

    def __init__(self):

        self.issue_submit_pub = rospy.Publisher('issue_server/issue_submit',Issue,queue_size=10)

        self.get_issues = rospy.ServiceProxy('issue_server/get_issues',GetIssues)
        self.clear_issue = rospy.ServiceProxy('issue_server/clear_issue',ClearIssue)
        self.get_pending_jobs = rospy.ServiceProxy('issue_server/get_pending_jobs',GetPendingJobs)
        self.set_pending_jobs = rospy.ServiceProxy('issue_server/set_pending_jobs',SetPendingJobs)
        self.clear_pending_job = rospy.ServiceProxy('issue_server/clear_pending_job',ClearPendingJob)

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
