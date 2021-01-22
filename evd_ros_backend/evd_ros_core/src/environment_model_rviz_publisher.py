#!/usr/bin/env python

import rospy
import pprint

from visualization_msgs.msg import Marker, MarkerArray
from interfaces.data_client_interface import DataClientInterface


DEFAULT_ROS_FRAME_ID = 'app'


class EnvironmentModelRvizPublisher:

    def __init__(self, ros_frame_id, display_traces):
        self.display_traces = display_traces
        self._count = 0
        self._marker_uuid_list = {}

        self._ros_frame_id = ros_frame_id
        self._marker_pub = rospy.Publisher('environment_model_visualizer/markers',MarkerArray,queue_size=10,latch=True)
        self._data_client = DataClientInterface(on_program_update_cb=self._update_markers)

    def _update_markers(self):

        print 'Environment'

        markerArray = MarkerArray()

        print '\n\n\n\n'
        print 'Cache Log'
        pprint.pprint(self._data_client.cache.utility_cache_stats())

        print '\n\n\n\nupdating markers'
        updated_markers = {}

        # get all locations and display all locations
        locations = self._data_client.program.environment.locations
        print '\n\nLocations=', locations
        for loc in locations:
            marker = loc.to_ros_marker(self._ros_frame_id,self._count)
            print 'adding location markers', loc.uuid
            markerArray.markers.append(marker)
            self._count += 1
            updated_markers[loc.uuid] = marker

        # Get all trajectories and display all waypoints and display all traces
        trajectories = self._data_client.cache.trajectories.values()
        print '\n\nTrajectories', trajectories
        for traj in trajectories:

            trajMarker, waypointMarkers, waypointUuids = traj.to_ros_markers(self._ros_frame_id, self._count)
            self._count = self._count + 1 + len(waypointMarkers)

            # waypoints
            for i in range(0,len(waypointMarkers)):
                marker = waypointMarkers[i]
                uuid = waypointUuids[i]
                print 'adding waypoint marker', uuid
                markerArray.markers.append(marker)
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
                        markerArray.markers.append(marker)
                        updated_markers[uuid] = marker

                    # trace line
                    print 'adding line trace marker', traj.trace.uuid
                    markerArray.markers.append(traceMarkers[i])
                    updated_markers[traj.trace.uuid] = traceMarkers[i]

            else:
                print 'adding trajectory line marker', traj.uuid
                markerArray.markers.append(trajMarker)
                updated_markers[traj.uuid] = trajMarker

        # Get all things
        things = self._data_client.cache.things.values()
        print '\n\nThings=', things
        for thing in things:
            marker = thing.to_ros_marker(self._ros_frame_id,self._count)
            if marker != None:
                print 'adding thing markers', thing.uuid
                markerArray.markers.append(marker)
                self._count = self._count + 1
                updated_markers[thing.uuid] = marker

        # display reach sphere
        reach_sphere = self._data_client.program.environment.reach_sphere
        print '\n\nReach Sphere', reach_sphere.uuid
        marker = reach_sphere.to_ros_marker(self._ros_frame_id,self._count)
        markerArray.markers.append(marker)
        self._count += 1
        updated_markers[reach_sphere.uuid] = marker

        # display occupancy zones
        zones = self._data_client.program.environment.occupancy_zones
        print '\n\nOccupancy Zones', zones
        for zone in zones:
            marker = zone.to_ros_marker(self._ros_frame_id, self._count)
            markerArray.markers.append(marker)
            self._count += 1
            updated_markers[zone.uuid] = marker

        # display pinch-points
        pinchpoints = self._data_client.program.environment.pinch_points
        print '\n\nPinch Points', pinchpoints
        for point in pinchpoints:
            marker  = point.to_ros_marker(self._count)
            markerArray.markers.append(marker)
            self._count += 1
            updated_markers[point.uuid] = marker

        # display collision meshes
        meshes = self._data_client.program.environment.collision_meshes
        print '\n\nCollision Meshes', meshes
        for mesh in meshes:
            marker = mesh.to_ros_marker(self._ros_frame_id, self._count)
            markerArray.markers.append(marker)
            self._count += 1
            updated_markers[mesh.uuid] = marker

        # Delete any markers that have not been updated
        for ids in self._marker_uuid_list.keys():
            if not ids in updated_markers.keys():
                marker = self._marker_uuid_list[ids]
                marker.action = Marker.DELETE
                print 'deleting marker', ids
                markerArray.markers.append(marker)

        # Update marker list
        self._marker_pub.publish(markerArray)
        self._marker_uuid_list = updated_markers


if __name__ == "__main__":
    rospy.init_node('environment_model_rviz_publisher')

    ros_frame_id = rospy.get_param('~ros_frame_id',DEFAULT_ROS_FRAME_ID)

    node = EnvironmentModelRvizPublisher(ros_frame_id, True)

    rospy.spin()
