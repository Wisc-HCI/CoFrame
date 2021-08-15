
from geometry_msgs.msg import Pose
from scipy.interpolate import interp1d
from pyquaternion import Quaternion


class PoseInterpolator:

    def __init__(self, poseStart, poseEnd, velocity, minTime=1):
        '''
        param: poseStart is starting pose (that robot should already be at)
        param: poseEnd is target ending pose
        param: velocity scalar for positional intepolation
        '''

        pos0 = poseStart.position
        pos1 = poseEnd.position

        # Compute positional timing
        tx = abs(pos1.x - pos0.x) / velocity
        ty = abs(pos1.y - pos0.y) / velocity
        tz = abs(pos1.z - pos0.z) / velocity

        # Generate interpolation functions
        self.xInterp = interp1d([0, tx], [pos0.x, pos1.x], kind='linear', assume_sorted=True, bounds_error=False, fill_value=(pos0.x, pos1.x))
        self.yInterp = interp1d([0, ty], [pos0.y, pos1.y], kind='linear', assume_sorted=True, bounds_error=False, fill_value=(pos0.y, pos1.y))
        self.zInterp = interp1d([0, tz], [pos0.z, pos1.z], kind='linear', assume_sorted=True, bounds_error=False, fill_value=(pos0.z, pos1.z))

        # Prepare quaternions for interpolation
        rot0 = poseStart.orientation
        self._q0 = Quaternion(rot0.w, rot0.x, rot0.y, rot0.z)
        rot1 = poseEnd.orientation
        self._q1 = Quaternion(rot1.w, rot1.x, rot1.y, rot1.z)

        # set final timing expectations
        if max([tx,ty,tz]) > minTime:
            self._fullTime = max([tx,ty,tz])
        else:
            self._fullTime = minTime # for rotation only movements

    @property
    def full_time(self):
        return self._fullTime

    def step(self, time):

        pose = Pose()
        pose.position.x = self.xInterp(time)
        pose.position.y = self.yInterp(time)
        pose.position.z = self.zInterp(time)

        t = self.clamp(time/self._fullTime, 0, 1)
        q = Quaternion.slerp(self._q0, self._q1, amount=time/self._fullTime)
        pose.orientation.w = q.w
        pose.orientation.x = q.x
        pose.orientation.y = q.y
        pose.orientation.z = q.z

        return pose

    def clamp(self, val, lower, upper):
        if val < lower:
            return lower
        elif val > upper:
            return upper
        else:
            return val