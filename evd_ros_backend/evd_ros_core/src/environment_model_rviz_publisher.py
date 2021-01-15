#!/usr/bin/env python

import rospy

from interfaces.data_client_interface import DataClientInterface
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA


DEFAULT_ROS_FRAME_ID = 'app'
DEFAULT_PUBLISH_RATE = 5.0

#TODO rewrite using the model-built-in marker generation

#TODO display collision meshes
#TODO display occupancy zones
#TODO display pinch-points
#TODO display reach sphere


class EnvironmentModelRvizPublisher:

    def __init__(self, ros_frame_id):
        self._count = 0
        self._marker_uuid_list = {}

        self._ros_frame_id = ros_frame_id
        self._marker_pub = rospy.Publisher('environment_model_visualizer/markers',Marker,queue_size=10,latch=True)
        self._data_client = DataClientInterface(use_environment_interface=True, on_environment_update_cb=self._update_markers)

    def _update_markers(self):
        print 'updating markers'
        updated_markers = {}

        # get all locations and display all locations
        locations = self._data_client.environment.locations.values()

        for loc in locations:
            marker = Marker()
            marker.header.frame_id = self._ros_frame_id
            marker.type = Marker.MESH_RESOURCE
            marker.ns = 'location'
            marker.id = self._count
            marker.pose = loc.to_ros()
            marker.scale = Vector3(0.1,0.1,0.1)
            marker.color = ColorRGBA(173/255.0,216/255.0,230/255.0,1)
            marker.mesh_resource = 'package://evd_ros_core/markers/SimpleGripperPhycon.stl'

            print 'adding location markers', loc.uuid
            self._marker_pub.publish(marker)
            self._count = self._count + 1
            updated_markers[loc.uuid] = marker

        # Getall trajectories
        # display all waypoints
        # display all traces
        trajectories = self._data_client.environment.cache.trajectories.values()

        for traj in trajectories:

            # waypoints
            for wp in traj.waypoints:
                marker = Marker()
                marker.header.frame_id = self._ros_frame_id
                marker.type = Marker.ARROW
                marker.ns = 'waypoints'
                marker.id = self._count
                marker.pose = wp.to_ros()
                marker.scale = Vector3(0.05,0.01,0.01)
                marker.color = ColorRGBA(123/255.0,104/255.0,238/255.0,1)

                print 'adding waypoint marker', wp.uuid
                self._marker_pub.publish(marker)
                self._count = self._count + 1
                updated_markers[wp.uuid] = marker

            if traj.trace != None:
                # trace path
                for key in traj.trace.data.keys():
                    lineMarker = Marker()
                    lineMarker.header.frame_id = self._ros_frame_id
                    lineMarker.type = Marker.LINE_STRIP
                    lineMarker.ns = 'trace'
                    lineMarker.id = self._count
                    self._count = self._count + 1
                    lineMarker.scale = Vector3(0.01,0.01,0.01)

                    # render point
                    for point in traj.trace.data[key]:
                        marker = Marker()
                        marker.header.frame_id = self._ros_frame_id
                        marker.type = Marker.SPHERE
                        marker.ns = 'renderpoints'
                        marker.id = self._count
                        marker.pose = point.to_ros()
                        marker.scale = Vector3(0.025,0.025,0.025)
                        marker.color = ColorRGBA(255/255.0,255/255.0,255/255.0,1)

                        lineMarker.points.append(marker.pose.position)

                        print 'adding render point marker', point.uuid
                        self._marker_pub.publish(marker)
                        self._count = self._count + 1
                        updated_markers[point.uuid] = marker

                    # trace path color based on group
                    if key == traj.trace.end_effector_path:
                        lineMarker.color = ColorRGBA(0/255.0,255/255.0,0/255.0,1) #end effector color
                    elif key in traj.trace.joint_paths:
                        lineMarker.color = ColorRGBA(0/255.0,0/255.0,255/255.0,1) # joint paths color
                    elif key in traj.trace.tool_paths:
                        lineMarker.color = ColorRGBA(255/255.0,0/255.0,0/255.0,1) # tool paths  color
                    else:
                        lineMarker.color = ColorRGBA(128/255.0,128/255.0,128/255.0,1) # other color

                    print 'adding line trace marker', traj.trace.uuid
                    self._marker_pub.publish(lineMarker)
                    updated_markers[traj.trace.uuid] = lineMarker

        # Delete any markers that have not been updated
        for ids in self._marker_uuid_list.keys():
            if not ids in updated_markers.keys():
                marker = self._marker_uuid_list[ids]
                marker.action = Marker.DELETE
                print 'deleting marker', ids
                self._marker_pub.publish(marker)



if __name__ == "__main__":
    rospy.init_node('environment_model_rviz_publisher')

    ros_frame_id = rospy.get_param('~ros_frame_id',DEFAULT_ROS_FRAME_ID)

    node = EnvironmentModelRvizPublisher(ros_frame_id)

    rospy.spin()
