#!/usr/bin/env python

import rospy
import pprint

from visualization_msgs.msg import Marker, MarkerArray
from interfaces.data_client_interface import DataClientInterface


DEFAULT_ROS_FRAME_ID = 'app'


class ProgramModelRvizPublisherNode:

    def __init__(self, ros_frame_id, display_traces):
        self.display_traces = display_traces
        self._count = 0
        self._marker_uuid_list = {}

        self._ros_frame_id = ros_frame_id
        self._marker_pub = rospy.Publisher('program_model_visualizer/markers',MarkerArray,queue_size=10,latch=True)
        self._data_client = DataClientInterface(on_program_update_cb=self._update_markers)

    def _update_markers(self):

        markerArray = MarkerArray()

        print '\n\n\n\n'
        print 'Cache Log'
        pprint.pprint(self._data_client.cache.utility_cache_stats())

        print '\n\n\n\nupdating markers'
        updated_markers = {}

        # get all locations and display all locations
        locations = self._data_client.cache.locations.values()
        print '\n\nLocations=', locations
        for loc in locations:
            marker = loc.to_ros_marker(self._ros_frame_id,self._count)
            print 'adding location markers', loc.uuid
            markerArray.markers.append(marker)
            self._count = self._count + 1
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
                print trajMarker

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
    rospy.init_node('program_model_rviz_publisher')

    ros_frame_id = rospy.get_param('~ros_frame_id',DEFAULT_ROS_FRAME_ID)

    node = ProgramModelRvizPublisherNode(ros_frame_id, True)

    rospy.spin()
