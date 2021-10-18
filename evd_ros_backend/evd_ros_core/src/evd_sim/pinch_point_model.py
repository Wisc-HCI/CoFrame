'''
Okay so here we are finally getting to pinch points

What are they they are a separation distance that is near the threat level for a human and can evolve over time
Effectively


So we are just using human hands as the pinch point guidance

'''




SEPERATOR = "___"

arm_combos = {
    "base_link_inertia": ["upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"],
    "shoulder_link": ["forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"],
    "upper_arm_link": ["wrist_1_link", "wrist_2_link", "wrist_3_link"],
    "forearm_link": ["wrist_2_link", "wrist_3_link"],
    "wrist_1_link": ["wrist_3_link"],
    "wrist_2_link": []
}

gripper = ["robotiq_85_base_link","robotiq_85_left_knuckle_link","robotiq_85_left_finger_link","robotiq_85_right_knuckle_link","robotiq_85_right_finger_link","robotiq_85_left_inner_knuckle_link","robotiq_85_left_finger_tip_link","robotiq_85_right_inner_knuckle_link","robotiq_85_right_finger_tip_link"]


def calc_center_point(point_a, point_b):
    val = []
    for i in range(0,len(point_a)):
        val.append((point_b[i] - point_a[i]) / 2)
    return val


def check_hand_thresholds(distance):
    # kinda emperical (from me) with conversative bounding
    return distance >= 0.025 and distance <= 0.06


def gen_semantic():
    semantic = {} # this is the structure I am interating over in analysis
    for a_link in arm_combos.keys():
        for b_link in arm_combos[a_link]:
            semantic["{}{}{}".format(a_link,SEPERATOR,b_link)] = (a_link,[b_link])
        
        semantic["{}{}{}".format(a_link,SEPERATOR,"gripper")] = (a_link, gripper)
    
    return semantic


def processPinchpoints(self_collision_data, length):
    semantic = gen_semantic()
    
    tracks = {}
    for key, (a_link, b_link_list) in semantic.items():
        tracks[key] = []

        for idx in range(0,length):

            candidate = None
            for b_link in b_link_list:
                obj = self_collision_data[a_link][b_link][idx]
                if check_hand_thresholds(obj['distance']):
                    candidate = {
                        'position': calc_center_point(obj['position_a'], obj['position_b']),
                        'gap': obj['distance']
                    }
                    break

            tracks[key].append(candidate)

    return tracks, semantic


def processPinchpoints_single(self_collision_data):
    semantic = gen_semantic()
    
    tracks = {}
    for key, (a_link, b_link_list) in semantic.items():
        tracks[key] = None
        for b_link in b_link_list:
            obj = self_collision_data[a_link][b_link]
            if check_hand_thresholds(obj['distance']):
                tracks[key] = {
                    'position': calc_center_point(obj['position_a'], obj['position_b']),
                    'gap': obj['distance']
                }
                break

    return tracks, semantic

'''
data = {
    "pybullet_self_collisions": {
        "<link_a>": {
            "<link_b>": [
                #...
                {
                    "position_a": (x, y, z),
                    "position_b": (x, y, z),
                    "distance": float
                }
                #...
            ]
        }
    }
}


output = {
    "pinchpoint_tracks": {

    },
    "pinchpoint_semantics": {
        # semantic structure
    }
}
'''