#!/usr/bin/env python3

import rospy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

class TFWorldPublisher:
    '''
    Listens to the '/tf' topic and republishes tranforms relative to world
    '''
    def __init__(self, default_frame):
        self.active_frames = []
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer)
        self.tf_publisher = rospy.Publisher('/tf_world',TFMessage,queue_size=10)
        self.tf_subscriber = rospy.Subscriber('/tf',TFMessage,self.check_tfs)
        self._default_frame = default_frame

    def check_tfs(self,msg):
        # This function really just checks the child frame ids and adds them to self.active_frames
        for transform in msg.transforms:
            child = transform.child_frame_id;
            if child not in self.active_frames:
                self.active_frames.append(child)

    def spin(self):
        i = 0
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():

            tfs = []

            for frame in self.active_frames:
                try:
                    trans = self.tf_buffer.lookup_transform(self._default_frame, frame, rospy.Time(0))
                    # print("LISTENER {0} {1}".format(frame,trans))
                    # tfstamped = TransformStamped(header=Header(seq=i,frame_id=self._default_frame,stamp= rospy.Time.now()),child_frame_id=frame,transform=trans)
                    tfs.append(trans)
                except (LookupException, ConnectivityException, ExtrapolationException):
                    continue

            self.tf_publisher.publish(TFMessage(transforms=tfs))
            i+=1
            rate.sleep()

            


if __name__ == "__main__":
    rospy.init_node('tf_world_publisher')

    try:
        ros_frame_id = rospy.get_param('~ros_frame_id',DEFAULT_ROS_FRAME_ID)
    except:
        ros_frame_id = 'world'

    node = TFWorldPublisher(ros_frame_id)

    node.spin()