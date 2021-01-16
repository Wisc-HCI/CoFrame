#!/usr/bin/env python

import rospy

from interfaces.data_client_interface import DataClientInterface


DEFAULT_ROS_FRAME_ID = 'app'
DEFAULT_PUBLISH_RATE = 5.0


class EnvironmentModelRvizPublisher:

    def __init__(self, ros_frame_id, display_traces):
        self.display_traces = display_traces
        self._count = 0
        self._marker_uuid_list = {}

        self._ros_frame_id = ros_frame_id
        self._marker_pub = rospy.Publisher('environment_model_visualizer/markers',Marker,queue_size=10,latch=True)
        self._data_client = DataClientInterface(use_environment_interface=True, on_environment_update_cb=self._update_markers)

    def _update_markers(self):
        print 'updating markers'
        updated_markers = {}

        # get all locations and display all locations
        locations = self._data_client.environment.locations
        for loc in locations:
            marker = loc.to_ros_marker(self._ros_frame_id,self._count)
            print 'adding location markers', loc.uuid
            self._marker_pub.publish(marker)
            self._count += 1
            updated_markers[loc.uuid] = marker

        # Get all trajectories and display all waypoints and display all traces
        trajectories = self._data_client.environment.trajectories
        for traj in trajectories:

            trajMarker, waypointMarkers, waypointUuids = traj.to_ros_markers(self._ros_frame_id, self._count)
            self._count = self._count + 1 + len(waypointMarkers)

            # waypoints
            for i in range(0,len(waypointMarkers)):
                marker = waypointMarkers[i]
                uuid = waypointUuids[i]
                print 'adding waypoint marker', uuid
                self._marker_pub.publish(marker)
                updated_markers[uuid] = marker

            # If we can and want to display traces otherwise just display the trajectory sketch
            if traj.trace != None and self.display_traces:
                traceMarkers, renderpointMarkers, renderpointUuids = traj.trace.to_ros_markers(self._ros_frame_id,self._count)
                self._count = self._count + len(traceMarkers) + len(renderpointMarkers)

                # For each trace
                for i in range(0,len(traceMarkers)):

                    #renderpoints
                    for j in range(0,len(renderpointMarkers[i])):
                        marker = renderpointMarkers[i][j]
                        uuid = renderpointUuids[i][j]

                        print 'adding render point marker', uuid
                        self._marker_pub.publish(marker)
                        updated_markers[uuid] = marker

                    # trace line
                    print 'adding line trace marker', traj.trace.uuid
                    self._marker_pub.publish(traceMarkers[i])
                    updated_markers[traj.trace.uuid] = traceMarkers[i]

            else:
                print 'adding trajectory line marker', traj.uuid
                self._marker_pub.publish(trajMarker)
                updated_markers[traj.uuid] = trajMarker

        # display reach sphere
        reach_sphere = self._data_client.environment.reach_sphere
        marker = reach_sphere.to_ros_marker('base_link',self._count)
        self._marker_pub.publish(marker)
        self._count += 1
        updated_markers[reach_sphere.uuid] = marker

        # display occupancy zones
        zones = self._data_client.environment.occupancy_zones
        for zone in zones:
            marker = zone.to_ros_marker(self._ros_frame_id, self._count)
            self._marker_pub.publish(marker)
            self._count += 1
            updated_markers[zone.uuid] = marker

        # display pinch-points
        pinchpoints = self._data_client.environment.pinch_points
        for point in pinchpoints:
            marker  = point.to_ros_marker(self._count)
            self._marker_pub.publish(marker)
            self._count += 1
            updated_markers[point.uuid] = marker

        # display collision meshes
        meshes = self._data_client.environment.collision_meshes
        for mesh in meshes:
            marker = mesh.to_ros_marker(self._ros_frame_id, self._count)
            self._marker_pub.publish(marker)
            self._count += 1
            updated_markers[mesh.uuid] = marker

        # Delete any markers that have not been updated
        for ids in self._marker_uuid_list.keys():
            if not ids in updated_markers.keys():
                marker = self._marker_uuid_list[ids]
                marker.action = Marker.DELETE
                print 'deleting marker', ids
                self._marker_pub.publish(marker)

        # Update marker list
        self._marker_uuid_list = updated_markers


if __name__ == "__main__":
    rospy.init_node('environment_model_rviz_publisher')

    ros_frame_id = rospy.get_param('~ros_frame_id',DEFAULT_ROS_FRAME_ID)

    node = EnvironmentModelRvizPublisher(ros_frame_id, True)

    rospy.spin()
