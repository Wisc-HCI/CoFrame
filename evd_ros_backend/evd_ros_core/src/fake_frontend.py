#!/usr/bin/env python3

'''
Mocked-up the expected interface between the javascript frontend and the ROS system.
'''

import time
import json
import rospy

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
from evd_ros_core.msg import Job, Issue, StringArray

from evd_script import Program, NodeParser, Pose
from evd_script.examples import CreateDebugApp


class FakeFrontendNode:

    def __init__(self, prefix='', rate=1, default_program=None):
        self._prefix = prefix
        prefix_fmt = prefix + '/' if prefix != '' else prefix
        self._rate = rate

        self._issues = {}
        if default_program == None:
            self._program = Program()
        else:
            self._program = default_program

        ## Publish these as PoseStamped, the backend will create TFs from these
        self._app_frame_pub = rospy.Publisher('application/app_to_ros_frame', PoseStamped, queue_size=5)
        self._camera_pose_pub = rospy.Publisher('application/camera_pose', PoseStamped, queue_size=5)
        self._control_target_pose_pub = rospy.Publisher('application/control_target_pose', PoseStamped, queue_size=5)

        ## Handle object registration. 
        # Object registration is how backend EvDScript objects get propogated to the frontend
        # and update/configure processors is how EvDScript in the frontend ends up in the backend
        # 1) Send out a call to register
        # 2) Receive all objects on registration
        # 3) (Optionally) Periodically publish full EvD program (useful for RViz right now)
        # 4) Publish all relevant EvDscript data to processors as JSON string of dict with fields
        #           - collision_meshes : []
        #           - pinch_points : []
        #           - occupancy_zones : []
        self._registration_pub = rospy.Publisher('{0}program/call_to_register'.format(prefix_fmt), Empty, queue_size=5)
        self._registration_sub = rospy.Subscriber('{0}program/register'.format(prefix_fmt), StringArray, self._program_register_cb)
        self._update_pub = rospy.Publisher('{0}program/update'.format(prefix_fmt), String, queue_size=5) #this is optional (I use it for visualization)
        self._configure_processors = rospy.Publisher('{0}program/configure/processors'.format(prefix_fmt), String, queue_size=5) # This is a json obj of all nodes needed for trace processing

        ## Communication with trace processor
        # This processor produces graded traces from trajectories.
        # 1) Publish JOB with data JSON string of form below and an ID
        # {
        #    "trajectory": <EVD Trajectory OBJ>,
        #    "points": [ //For all locations and waypoints used by trajectory
        #       <EVD Waypoint>, ....,
        #       <EvD Location>, ...
        #    ]     
        # }
        # 2) Receive trace data as JSON string or null if processor unable to compute trace (its packed as a Job)
        # 3) (Optionally) cancel the job with its ID
        self._trace_request_pub = rospy.Publisher('{0}program/request/trace'.format(prefix_fmt), Job, queue_size=5)
        self._trace_submit_sub = rospy.Subscriber('{0}program/submit/trace'.format(prefix_fmt), Job, self._trace_submit_cb)
        self._trace_clear_pub = rospy.Publisher('{0}program/clear/trace'.format(prefix_fmt), String, queue_size=5)

        ## Communication with joint processor
        # This processor produces joints EvD objects from waypoints or locations
        # 1) Publish Job with data JSON string of waypoint or location
        # 2) Receive joints data JSON string or null if processor unable to compute
        # 3) (Optionally) cancel the job with its ID
        self._joints_request_pub = rospy.Publisher('{0}program/request/joints'.format(prefix_fmt), Job, queue_size=5)
        self._joints_submit_sub = rospy.Subscriber('{0}program/submit/joints'.format(prefix_fmt), Job, self._joints_submit_cb)
        self._joints_clear_pub = rospy.Publisher('{0}program/clear/joints'.format(prefix_fmt), String, queue_size=5)

        # NOTE: this is just for the fake frontend to publish periodic updates
        self._timer = rospy.Timer(rospy.Duration(0.5), self._update_cb)

    def _program_register_cb(self, msg):
        for entry in msg.data:
            node = NodeParser(json.loads(entry))
            self._program.add_child(node)

    def _trace_submit_cb(self, msg):
        pass

    def _joints_submit_cb(self, msg):
        pass

    def _issue_submit_cb(self, msg):
        self._issues[msg.id] = msg

    def _issue_clear_cb(self, msg):
        if msg.id in self._issues.keys():
            del self._issues[msg.id]
        else:
            pass # trying to delete non-existent issue (ignore)

    def _update_cb(self, event=None):
        data = json.dumps(self._program.to_dct())
        self._update_pub.publish(String(data))

    def spin(self):

        rospy.sleep(2.5) #seconds

        # At some point the frontend should define the `app` transform and then two poses from that.
        # `world` (which is the ROS root) will be relative to `app`
        self._app_frame_pub.publish(Pose(Position.Zero(),Orientation.Identity(), link='app').to_ros(stamped=True))
        self._camera_pose_pub.publish(Pose(Position.from_axis('x'),Orientation.Identity(), link='app').to_ros(stamped=True))
        self._control_target_pose_pub.publish(Pose(Position.Zero(),Orientation.Identity(), link='app').to_ros(stamped=True))

        #request registration from the backend (this gets information known about the setup)
        self._registration_pub.publish(Empty()) 

        # Push out the current aggregated state of the environment ot the processors
        # (This should happen whenever there is a change to these)
        # (Note that pushing these will invalidate any pending jobs plus you should clear out any
        #  previously processed results so update this with caution)
        msg = String()
        msg.data = json.dumps({
            'collision_meshes': [x.to_dct() for x in self._program.environment.collision_meshes],
            'pinch_points': [x.to_dct() for x in self._program.environment.pinch_points],
            'occupancy_zones': [x.to_dct() for x in self._program.environment.occupancy_zones]
        })
        self._configure_processors.publish(msg)

        # now that the backend is fully set up do what ever the hell you want in the frontend
        start = time.time()
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():

            # Note: update processors if the relevant environment has changes
            # Note: update the control target if the interactive marker is moved
            # Note: update the camera pose if the user moves it around
            # Note: send out requests to the processors
            # Note: handle issues

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('fake_frontend')

    prefix = rospy.get_param('~prefix','')
    rate = rospy.get_param('~rate',1)
    load_debug_prog = rospy.get_param('~load_debug_program',False)

    default_prog = CreateDebugApp() if load_debug_prog else None

    node = FakeFrontendNode(prefix,rate,default_prog)
    node.spin()