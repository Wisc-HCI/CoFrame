#!/usr/bin/env python3

'''
Tests the environment behavior of EvD with an RViz Visualization.

Checks:
- Locations
- Trajectories
- Waypoints
- Things
- ReachSphere
- Occupancy Zones
- Pinch Points
- Collision Meshes
'''


import rospy

from evd_interfaces.frontend_interface import FrontendInterface
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_ROS_FRAME_ID = 'app'


class TestDataVisualizer:

    def __init__(self, ros_frame_id):
        self._count = 0
        self._marker_uuid_list = {}

        self._ros_frame_id = ros_frame_id
        self._marker_pub = rospy.Publisher('data_visualizer/markers',MarkerArray,queue_size=10,latch=True)
        
        self._program = None
        self._program_inferface = FrontendInterface(use_update=True, update_cb=self._update_markers)

    def _update_markers(self, program):
        self._program = program

    def spin(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():

            rate.sleep()
            if self._program == None:
                continue

            markerArray = MarkerArray()
            updated_markers = {}

            # Data Nodes
            self.__render_locations(markerArray,updated_markers)
            self.__render_things(markerArray,updated_markers)
            self.__render_trajectories(markerArray,updated_markers)
            self.__render_traces(markerArray,updated_markers)

            # Environment Nodes
            self.__render_reach_sphere(markerArray,updated_markers)
            self.__render_collision_meshes(markerArray,updated_markers)
            self.__render_occupancy_zones(markerArray,updated_markers)
            self.__render_pinch_points(markerArray,updated_markers)

            # Update
            self.__delete_old_markers(markerArray,updated_markers)
            self.__publish_markers(markerArray,updated_markers)       

    def __render_trajectories(self, markerArray, updated_markers):
        # Get all trajectories and display all waypoints
        trajectories = self._program.environment.trajectories
        #print '\n\nTrajectories', trajectories
        for traj in trajectories:

            trajMarker, waypointMarkers, waypointUuids = traj.to_ros_markers(self._ros_frame_id, self._count)
            self._count = self._count + 1 + len(waypointMarkers)

            # waypoints
            for i in range(0,len(waypointMarkers)):
                marker = waypointMarkers[i]
                uuid = waypointUuids[i]
                #print 'adding waypoint marker', uuid
                markerArray.markers.append(marker)
                updated_markers[uuid] = marker

            markerArray.markers.append(trajMarker)
            updated_markers[traj.uuid] = trajMarker

    def __render_traces(self, markerArray, updated_markers):
        # Get all trajectories and display all waypoints
        trajectories = self._program.environment.trajectories
        #print '\n\nTrajectories', trajectories
        for traj in trajectories:
            if traj.trace != None:
                traceMarkers, renderpointMarkers, renderpointUuids = traj.trace.to_ros_markers(self._ros_frame_id,self._count)
                self._count = self._count + len(traceMarkers) + len(renderpointMarkers)

                # For each trace
                for i in range(0,len(traceMarkers)):

                    #renderpoints
                    for j in range(0,len(renderpointMarkers[i])):
                        marker = renderpointMarkers[i][j]
                        uuid = renderpointUuids[i][j]

                        #print 'adding render point marker', uuid
                        markerArray.markers.append(marker)
                        updated_markers[uuid] = marker

                    # trace line
                    #print 'adding line trace marker', traj.trace.uuid
                    markerArray.markers.append(traceMarkers[i])
                    updated_markers[traj.trace.uuid] = traceMarkers[i]

    def __render_pinch_points(self, markerArray, updated_markers):
        # display pinch-points
        pinchpoints = self._program.environment.pinch_points
        #print '\n\nPinch Points', pinchpoints
        for point in pinchpoints:
            marker  = point.to_ros_marker(self._count)
            markerArray.markers.append(marker)
            self._count += 1
            updated_markers[point.uuid] = marker

    def __render_collision_meshes(self, markerArray, updated_markers):
        # display collision meshes
        meshes = self._program.environment.collision_meshes
        #print '\n\nCollision Meshes', meshes
        for mesh in meshes:
            marker = mesh.to_ros_marker(self._count)
            markerArray.markers.append(marker)
            self._count += 1
            updated_markers[mesh.uuid] = marker

    def __render_locations(self, markerArray, updated_markers):
        # get all locations and display all locations
        locations = self._program.environment.locations
        #print '\n\nLocations=', locations
        for loc in locations:
            marker = loc.to_ros_marker(self._ros_frame_id,self._count)
            #print 'adding location markers', loc.uuid
            markerArray.markers.append(marker)
            self._count += 1
            updated_markers[loc.uuid] = marker

    def __render_things(self, markerArray, updated_markers):
        # Get all things
        things = self._program.environment.things
        #print '\n\nThings=', things
        for thing in things:
            marker = thing.to_ros_marker(self._ros_frame_id,self._count)
            if marker != None:
                #print 'adding thing markers', thing.uuid
                markerArray.markers.append(marker)
                self._count = self._count + 1
                updated_markers[thing.uuid] = marker

    def __render_reach_sphere(self, markerArray, updated_markers):
        # display reach sphere
        reach_sphere = self._program.environment.reach_sphere
        if reach_sphere != None:
            #print '\n\nReach Sphere', reach_sphere.uuid
            marker = reach_sphere.to_ros_marker(self._ros_frame_id,self._count)
            markerArray.markers.append(marker)
            self._count += 1
            updated_markers[reach_sphere.uuid] = marker

    def __render_occupancy_zones(self, markerArray, updated_markers):
        # display occupancy zones
        zones = self._program.environment.occupancy_zones
        #print '\n\nOccupancy Zones', zones
        for zone in zones:
            marker = zone.to_ros_marker(self._ros_frame_id, self._count)
            markerArray.markers.append(marker)
            self._count += 1
            updated_markers[zone.uuid] = marker

    def __delete_old_markers(self, markerArray, updated_markers):
        # Delete any markers that have not been updated
        for uuid in self._marker_uuid_list.keys():
            if not uuid in updated_markers.keys():
                marker = self._marker_uuid_list[uuid]
                marker.action = Marker.DELETE
                markerArray.markers.append(marker)
            else:
                marker_old = self._marker_uuid_list[uuid]
                marker_new = updated_markers[uuid]

                if marker_old.id != marker_new.id:
                    marker_old.action = Marker.DELETE
                    markerArray.markers.append(marker_old)

    def __publish_markers(self, markerArray, updated_markers):
        # Update marker list
        self._marker_pub.publish(markerArray)
        self._marker_uuid_list = updated_markers

    
if __name__ == "__main__":
    rospy.init_node('test_environment_data_visualizer')

    ros_frame_id = rospy.get_param('~ros_frame_id',DEFAULT_ROS_FRAME_ID)

    node = TestDataVisualizer(ros_frame_id)

    node.spin()