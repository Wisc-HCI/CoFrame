  

def joints_reached(joints, targets, threshold):
    if len(joints) != len(targets):
        raise Exception('Lengths must match')

    if not isinstance(threshold,list):
        threshold = [threshold] * len(joints)
    
    if len(threshold) != len(joints):
        raise Exception('Lengths must match')

    reached = True
    for j, t, v in zip(joints, targets, threshold):
        reached = reached and joint_reached(j,t,v)
    
    return reached

def joint_reached(joint, target, threshold):
    return abs(joint - target) < threshold