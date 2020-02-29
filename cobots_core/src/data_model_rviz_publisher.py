#!/usr/bin/env python

from interfaces.data_client_interface import DataClientInterface
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA


DEFAULT_ROS_FRAME_ID = 'world'
DEFAULT_PUBLISH_RATE = 5.0


class DataModelRvizPublisherNode:

    def __init__(self, ros_frame_id):
        self._count = 0
        self._marker_uuid_list = []

        self._ros_frame_id = ros_frame_id
        self._tf_br = tf.TransformBroadcaster()
        self._data_client = DataClientInterface(use_program_interface=True, on_program_update_cb=self._update_markers)
        self._marker_pub = rospy.Publisher('data_model_visualizer/markers',Marker,queue_size=10)

    def _update_markers(self):
        updated_markers = []

        # get all locations and display all locations
        locations = self._data_client.program.find_all_locations()

        for loc in locations:
            marker = Marker()
            marker.header.frame_id = self._ros_frame_id
            marker.type = Marker.MESH_RESOURCE
            marker.id = self._count
            marker.pose = loc.to_ros()
            marker.scale = Vector3(1,1,1)
            marker.color = ColorRGBA(173/255.0,216/255.0,230/255.0,1)
            marker.mesh_resource = 'package://cobots_core/markers/SimpleGripperPhycon.stl'

            self._marker_pub.publish(marker)
            self._count = self._count + 1
            updated_markers.append(loc.uuid)

        # Get all trajectories
        # display all waypoints
        # display all traces
        trajectories = self._data_client.program.find_all_trajectories()

        for traj in trajectories:

            # waypoints
            for wp in traj.waypoints:
                marker = Marker()
                marker.header.frame_id = self._ros_frame_id
                marker.type = Marker.ARROW
                marker.id = self._count
                marker.pose = wp.to_ros()
                marker.scale = Vector3(1,1,1)
                marker.color = ColorRGBA(123/255.0,104/255.0,238/255.0,1)

                self._marker_pub.publish(marker)
                self._count = self._count + 1
                updated_markers.append(wp.uuid)

            if traj.trace != None:

                # trace path
                for key in traj.trace.data.keys():
                    lineMarker = Marker()
                    lineMarker.header.frame_id = self._ros_frame_id
                    lineMarker.type = Marker.LINE_STRIP
                    lineMarker.id = self._count
                    lineMarker.scale = Vector3(1,1,1)

                    # render point
                    for point in traj.trace.data[key]:
                        marker = Marker()
                        marker.header.frame_id = self._ros_frame_id
                        marker.type = Marker.SPHERE
                        marker.id = self._count
                        marker.pose = point.to_ros()
                        marker.scale = Vector3(1,1,1)
                        marker.color = ColorRGBA(255/255.0,255/255.0,255/255.0,1)

                        lineMarker.points.append(marker.pose.position)

                        self._marker_pub.publish(marker)
                        self._count = self._count + 1
                        updated_markers.append(point.uuid)

                    # trace path color based on group
                    if key == traj.trace.end_effector_path:
                        lineMarker.color = ColorRGBA(0/255.0,255/255.0,0/255.0,1) #end effector color
                    elif key in traj.trace.joint_paths:
                        lineMarker.color = ColorRGBA(0/255.0,0/255.0,255/255.0,1) # joint paths color
                    elif key in traj.trace.tool_paths:
                        lineMarker.color = ColorRGBA(255/255.0,0/255.0,0/255.0,1) # tool paths  color
                    else:
                        lineMarker.color = ColorRGBA(128/255.0,128/255.0,128/255.0,1) # other color

                    self._marker_pub.publish(lineMarker)
                    self._count = self._count + 1
                    updated_markers.append(traj.trace.uuid)

        # Delete any markers that have not been updated
        for ids in self._marker_uuid_list:
            if not ids in updated_markers:
                pass #TODO need to delete old markers that are not being used


if __name__ == "__main__":
    rospy.init_node('data_model_rviz_publisher')

    ros_frame_id = rospy.get_param('~ros_frame_id',DEFAULT_ROS_FRAME_ID)

    node = DataModelRvizPublisherNode(ros_frame_id)

    rospy.spin()
