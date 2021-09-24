import math

from scipy.interpolate import interp1d


class JointInterpolator:
    
    def __init__(self, joints, velocity):
        '''
        param: joints is list of joint lists [[j0_0, j0_1,...],[j1_0,...],...] in units ja
        param: velocity is a list of scalars in units ja / sec
        '''

        if type(velocity) == int or type(velocity) == float:
            velocity = [velocity]*len(joints)

        # Compute raw times needed to hit joint states
        times = []
        for v, j in zip(velocity, joints):
            t = [0] # zero here for initial state

            for i in range(1,len(j)):
                dist = abs(j[i] - j[i-1])
                t.append(dist / v)
            
            times.append(t)

        self._fullTime = max(times)

        # Generate interpolation functions  
        self._interpFnts = []
        for t, j in zip(times, joints):
            self._interpFnts.append(interp1d(t, j, kind='linear', assume_sorted=True, bounds_error=False, fill_value=(j[0],j[-1])))

        # Define start
        self._start_joints = []
        self._end_joints = []
        for j in joints:
            self._start_joints.append(j[0])
            self._end_joints.append(j[len(j)-1])
        
    @property
    def full_time(self):
        return self._fullTime

    @property
    def start_joints(self):
        return self._start_joints

    @property
    def end_joints(self):
        return self._end_joints

    def step(self, time):
        js = []
        for f in self._interpFnts:
            js.append(f(time))
            
        return js