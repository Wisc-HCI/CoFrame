
import math


def jointsReached(joints, targets, threshold):
    if len(joints) != len(targets):
        raise Exception('Lengths must match')

    if not isinstance(threshold,list):
        threshold = [threshold] * len(joints)
    
    if len(threshold) != len(joints):
        raise Exception('Lengths must match')

    reached = True
    for j, t, v in zip(joints, targets, threshold):
        reached = reached and jointReached(j,t,v)
    
    return reached

def jointReached(joint, target, threshold):
    return abs(joint - target) < threshold