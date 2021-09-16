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

from evd_script import Program, NodeParser, Pose, Position, Orientation, get_evd_cache_obj
from evd_script.examples import CreateDebugApp


evd_cache = get_evd_cache_obj()


class FakeFrontendNode:

    def __init__(self, prefix='', rate=1, default_program=None):
        self._prefix = prefix
        prefix_fmt = prefix + '/' if prefix != '' else prefix
        self._rate = rate

        print('\n\n\n')
        print('STARTING FAKE FRONTEND')

        if default_program == None:
            print('empty program')
            self._program = Program()
        else:
            print('default program')
            self._program = default_program

        self._active_joints_jobs = []
        self._active_trace_jobs = []

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
        # 5) Publish all machine Evdscript data to machine templates
        self._registration_pub = rospy.Publisher('{0}program/call_to_register'.format(prefix_fmt), Empty, queue_size=5)
        self._registration_sub = rospy.Subscriber('{0}program/register'.format(prefix_fmt), StringArray, self._program_register_cb)
        self._update_pub = rospy.Publisher('{0}program/update'.format(prefix_fmt), String, queue_size=5) #this is optional (I use it for visualization)
        self._configure_processors = rospy.Publisher('{0}program/configure/processors'.format(prefix_fmt), String, queue_size=5) # This is a json obj of all nodes needed for trace processing
        self._configure_machines = rospy.Publisher('{0}program/configure/machines'.format(prefix_fmt), String, queue_size=5)

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
        print('In register callback, we got')
        for entry in msg.data:
            print(entry)
            node = NodeParser(json.loads(entry))
            self._program.add_child(node)
        print('/n/n')

    def _trace_submit_cb(self, msg):
        if msg.id in self._active_trace_jobs:

            trace = NodeParser(json.loads(msg.data))
            traj = evd_cache.get(msg.id) # where job ID is the uuid of the parent object (only works for 1-1 relations). Caution if trying to have multiple traces for a single trajectory.
            traj.trace = trace

            self._active_trace_jobs.remove(msg.id)

    def _joints_submit_cb(self, msg):
        if msg.id in self._active_joints_jobs:

            joints = NodeParser(json.loads(msg.data))
            wp = evd_cache.get(msg.id) # where job ID is the uuid of the parent object (only works for 1-1 relations)
            wp.joints = joints

            self._active_joints_jobs.remove(msg.id)

    def _update_cb(self, event=None):
        #print('\n\n\tIn periodic update push!\n\n')
        data = json.dumps(self._program.to_dct())
        self._update_pub.publish(String(data))

    def spin(self):

        print('Going to wait for 2.5 seconds')
        rospy.sleep(2.5) #seconds

        print('publishing our app frame linkages')
        # At some point the frontend should define the `app` transform and then two poses from that.
        # In Unity the `world` (which is the ROS root) would be relative to `app` (so it would have to be reversed first)
        self._app_frame_pub.publish(Pose(Position.Zero(),Orientation.Identity(), link='world').to_ros(stamped=True))            # This defines the App frame
        self._camera_pose_pub.publish(Pose(Position.from_axis('x'),Orientation.Identity(), link='app').to_ros(stamped=True))    # This defines the visualization_camera frame
        self._control_target_pose_pub.publish(Pose(Position.Zero(),Orientation.Identity(), link='app').to_ros(stamped=True))    # This defines the control_target frame

        print('Calling nodes to register')
        #request registration from the backend (this gets information known about the setup)
        self._registration_pub.publish(Empty()) 

        print('Going to wait for 5 seconds to register everythng')
        rospy.sleep(5)

        print('Push registered env objs to processors')
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

        print('Push registered machines to machine implementations')
        # Push out the current aggregated state of registered machines to their respective implementation
        msg = String()
        msg.data = json.dumps({
            'machines': [x.to_dct() for x in self._program.environment.machines]
        })
        self._configure_machines.publish(msg)

        print('Going to wait for 5 seconds for our configuration changes to percolate')
        rospy.sleep(5)

        print('Starting fake frontend main loop')
        # now that the backend is fully set up, do what ever the hell you want in the frontend
        start = time.time()
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():

            # Note: update processors if the relevant environment has changes
            # Note: update the control target if the interactive marker is moved
            # Note: update the camera pose if the user moves it around
            # Note: send out requests to the processors

            self.create_joint_jobs()
            self.create_trace_jobs()

            rate.sleep()
    
    def create_joint_jobs(self):
        data = []
        data.extend(self._program.environment.locations)
        data.extend(self._program.environment.waypoints)

        for wp in data:
            if wp.joints == None and wp.uuid not in self._active_joints_jobs:
                self._active_joints_jobs.append(wp.uuid)
                
                job = Job()
                job.id= wp.uuid
                job.data = json.dumps({'point': wp.to_dct()})
                
                self._joints_request_pub.publish(job)

    def create_trace_jobs(self):
        data = self._program.environment.trajectories

        for traj in data:
            if traj.start_location_uuid != None and traj.end_location_uuid != None and traj.trace == None:
                self._active_trace_jobs.append(traj.uuid)

                job = Job()
                job.id = traj.uuid
                job.data = json.dumps({
                    'points': [ # Either package up just the points needed or if lazy just pass the whole set
                        self._program.environment.get_location(traj.start_location_uuid),
                        self._program.environment.get_location(traj.end_location_uuid)
                    ] + [ self._program.environment.get_waypoint(wp_uuid) for wp_uuid in traj.waypoint_uuids ],
                    'trajectory': traj.to_dct()
                })

                self._trace_request_pub.publish(job)


if __name__ == "__main__":
    rospy.init_node('fake_frontend')

    prefix = rospy.get_param('~prefix','')
    rate = rospy.get_param('~rate',1)
    load_debug_prog = rospy.get_param('~load_debug_program',False)

    default_prog = CreateDebugApp() if load_debug_prog else None

    node = FakeFrontendNode(prefix,rate,default_prog)
    node.spin()