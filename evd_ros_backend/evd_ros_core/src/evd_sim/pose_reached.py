
from pyquaternion import Quaternion


DEFAULT_POSITION_THRESHOLD = 0.05
DEFAULT_ORIENTATION_THRESHOLD = 0.05


def poseReached(p0, p1, 
                positionThreshold=DEFAULT_POSITION_THRESHOLD, 
                orientationThreshold=DEFAULT_ORIENTATION_THRESHOLD):

    posReached = positionReached(p0.position,p1.position,positionThreshold)
    ortReached = orientationReached(p0.orientation,p1.orientation)
    return posReached and ortReached

def positionDeltas(pos0, pos1):
    dx = abs(pos1.x - pos0.x)
    dy = abs(pos1.y - pos0.y)
    dz = abs(pos1.z - pos0.z)

    return (dx, dy, dz)

def positionReached(pos0, pos1, 
                    positionThreshold=DEFAULT_POSITION_THRESHOLD):
    (dx, dy, dz) = positionDeltas(pos0, pos1)

    xReached = dx < positionThreshold
    yReached = dy < positionThreshold
    zReached = dz < positionThreshold
    
    return xReached and yReached and zReached

def orientationDelta(rot0, rot1): 
    q0 = Quaternion(w=rot0.w, x=rot0.x, y=rot0.y, z=rot0.z)
    q1 = Quaternion(w=rot1.w, x=rot1.x, y=rot1.y, z=rot1.z)
    dist = Quaternion.absolute_distance(q0, q1)
    return dist

def orientationReached(rot0, rot1, 
                       orientationThreshold=DEFAULT_ORIENTATION_THRESHOLD): 
    dist = orientationDelta(rot0, rot1)
    return dist < orientationThreshold

def deltaDebug(p0, p1):
    posReached = positionDeltas(p0.position,p1.position)
    ortReached = orientationDelta(p0.orientation,p1.orientation)
    return {"position": posReached, "orientation": ortReached}