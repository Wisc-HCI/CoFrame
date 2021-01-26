
import rospy

from evd_ros_core.msg import Issue
from evd_ros_core.srv import GetIssues, GetIssuesRequest, GetIssuesResponse
from evd_ros_core.srv import ClearIssue, ClearIssueRequest, ClearIssueResponse
from evd_ros_core.srv import GetPendingJobs, GetPendingJobsRequest, GetPendingJobsResponse
from evd_ros_core.srv import SetPendingJobs, SetPendingJobsRequest, SetPendingJobsResponse
from evd_ros_core.srv import ClearPendingJob, ClearPendingJobRequest, ClearPendingJobResponse


class IssueClientInterface:

    def __init__(self):

        self.issue_pub = rospy.Publisher('issue_server/issue_submit',Issue,queue_size=10)
        
        self.get_issue_srv = rospy.ServiceProxy('issue_server/get_issues',GetIssues)
        self.clear_issue_srv = rospy.ServiceProxy('issue_server/clear_issue',ClearIssue)
        self.get_pending_jobs_srv = rospy.ServiceProxy('issue_server/get_pending_jobs',GetPendingJobs)
        self.set_pending_jobs_srv = rospy.ServiceProxy('issue_server/set_pending_jobs',SetPendingJobs)
        self.clear_pending_job_srv = rospy.ServiceProxy('issue_server/clear_pending_job',ClearPendingJob)
