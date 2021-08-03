import math

from scipy.interpolate import interp1d


class JointInterpolator:
    
    def __init__(self, joints, velocity):
        '''
        param: joints is list of joint lists [[j0_0, j0_1,...],[j1_0,...],...] in units ja
        param: velocity is a list of scalars in units ja / sec
        '''

        # Compute raw times needed to hit joint states
        times = []
        for v, j in zip(velocity, joints):
            t = [0] # zero here for initial state

            for i in range(1,len(j)):
                dist = abs(j[i] - j[i-1])
                t.append(dist / v)
            
            times.append(t)

        # Generate interpolation functions  
        self._interpFnts = []
        for t, j in zip(times, joints):
            self._interpFnts.append(interp1d(t, j, kind='cubic', assume_sorted=True, bounds_error=False, fill_value=(j[0],j[-1])))

    def step(self, time):
        js = []
        for f in self._interpFnts:
            js.append(f(time))
            
        return js