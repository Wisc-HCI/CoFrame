#!/usr/bin/env python3

import yaml
import rospy
import json
from std_msgs.msg import String, Bool
from lively_tk import ObjectiveInput, Solver, parse_config_data
from datetime import datetime

CONFIG_MATCH_FIELDS = {'ee_fixed_joints', 'static_environment','fixed_frame','joint_names', 'joint_ordering'}

def ewma(target,last=None,a=0.6):
    if last:
        return target * a + (1.0-a) * last
    else:
        return target

class Time(object):
    def __init__(self,sec,nanosec):
        self.sec = sec
        self.nanosec = nanosec

class SolverNode(object):
    def __init__(self):
        with open(rospy.get_param('~config_file')) as handle:
            self.config_data = yaml.safe_load(handle)
        
        self.config = parse_config_data(self.config_data)
        try:
            self.solver = Solver(self.config)
        except:
            self.solver = None

        # Services and Topics
        self.config_sub = rospy.Subscriber('/lively_ik/config_updates', String, self.handle_config_update, queue_size=10)
        self.weight_sub = rospy.Subscriber('/lively_ik/weight_updates', String, self.handle_weight_update, queue_size=10)
        self.direct_sub = rospy.Subscriber('/lively_ik/direct_updates', String, self.handle_direct_update, queue_size=10)
        self.enable_sub = rospy.Subscriber('/lively_ik/enabled_updates', Bool, self.handle_enabled_update, queue_size=10)
        self.refresh_sub = rospy.Subscriber('/lively_ik/refresh_solver', String, self.handle_refresh_update, queue_size=10)
        self.result_pub = rospy.Publisher('/lively_ik/result', String, queue_size=10)

        self.enabled = False

        # self.create_timer(5,self.standard_loop)
        self.iterations = 15*(3+len(self.config_data['starting_config'][1]))

        self.base_transform = self.config_data['starting_config'][0]
        self.displayed_state = self.config_data['starting_config'][1]
        self.target_weights = []
        self.current_weights = []
        self.target_directions = []
        self.current_directions = []

        rospy.loginfo('Initialized!')


    def handle_config_update(self,msg):
        rospy.logdebug('Received request to update config')
        data = json.loads(msg.data)
        try:
            if self.config_data == data:
                pass
            else:
                rospy.loginfo('Changing config')
                # is_similar = True
                # for field in CONFIG_MATCH_FIELDS:
                #     if self.config_data[field] != data[field]:
                #         is_similar = False
                # if is_similar and data['mode_control'] == 'absolute':
                #     data['starting_config'] = [self.base_transform,self.displayed_state]
                self.config_data = data
                self.iterations = 15*(3+len(self.config_data['starting_config'][1]))

                self.config = parse_config_data(data)
                self.current_weights = self.config.default_weights
                self.target_weights = self.current_weights
                try:
                    self.solver = Solver(self.config)
                    # Parse the rust goal specs into a python dictionary
                    current_directions = []
                    for default_goal in self.config.default_goals:
                        goal_data = {}
                        for field in ['scalar','vector','quaternion']:
                            value = getattr(default_goal,field)
                            if value:
                                goal_data[field] = value
                        current_directions.append(goal_data)

                    self.current_directions = current_directions
                    self.target_directions = self.current_directions
                    self.enabled = True
                    rospy.loginfo('Updated config succesfully. Enabling...')
                except:
                    self.solver = None
                    self.enabled = False
                    rospy.logwarn('Error updating solver. Disabling...')


        except Exception as e:
            rospy.logwarn('Invalid config supplied: {0}'.format(e))

    def handle_refresh_update(self,msg):
        self.config_data = json.loads(msg.data)
        self.iterations = 15*(3+len(self.config_data['starting_config'][1]))

        self.config = parse_config_data(self.config_data)
        self.current_weights = self.config.default_weights
        self.target_weights = self.current_weights
        try:
            self.solver = Solver(self.config)
            # Parse the rust goal specs into a python dictionary
            current_directions = []
            for default_goal in self.config.default_goals:
                goal_data = {}
                for field in ['scalar','vector','quaternion']:
                    value = getattr(default_goal,field)
                    if value:
                        goal_data[field] = value
                current_directions.append(goal_data)

            self.current_directions = current_directions
            self.target_directions = self.current_directions
            self.enabled = True
            rospy.loginfo('Updated config succesfully. Enabling...')
        except:
            self.solver = None
            self.enabled = False
            rospy.logwarn('Error updating solver. Disabling...')

    def handle_enabled_update(self,msg):
        rospy.logdebug('Received request to update enabled setting ({0}->{1})'.format(self.enabled,msg.data))
        if msg.data != self.enabled:
            rospy.logwarn('Changing enabled to {0}'.format(msg.data))
            self.enabled = msg.data

    def handle_weight_update(self,msg):
        rospy.logdebug('Received request to update weights')
        data = json.loads(msg.data)
        if len(data) == len(self.target_weights):
            self.target_weights = data

    def handle_direct_update(self,msg):
        rospy.logdebug('Received request to update directions')
        data = json.loads(msg.data)
        if len(data) == len(self.target_directions):
            self.target_directions = data

    def update_current_weights(self):
        # If the weights were changed in size, just replace and don't interpolate.
        if len(self.current_weights) != len(self.target_weights):
            self.current_weights = self.target_weights
        elif self.current_weights != self.target_weights:
            for idx,target_weight in enumerate(self.target_weights):
                # Use ewma to interpolate to new values
                self.current_weights[idx] = ewma(target_weight,self.current_weights[idx],0.05)

    def update_current_directions(self):
        # If the weights were changed in size, just replace and don't interpolate.
        if len(self.current_directions) != len(self.target_directions):
            self.current_directions = self.target_directions
        elif self.current_directions != self.target_directions:
            for idx,target_direction in enumerate(self.target_directions):
                # Use ewma to interpolate to new values
                current_direction = self.current_directions[idx]
                for key in current_direction.keys():
                    if key not in target_direction.keys():
                        self.current_directions[idx] = target_direction
                    elif key == 'scalar':
                        self.current_directions[idx]['scalar'] = ewma(target_direction['scalar'],current_direction['scalar'],0.05)
                    else:
                        for dim in range(len(current_direction[key])):
                            self.current_directions[idx][key][dim] = ewma(target_direction[key][dim],current_direction[key][dim],0.05)

    def standard_loop(self):
        if self.enabled and self.solver != None:
            self.update_current_weights()
            self.update_current_directions()
            self.base_transform, self.displayed_state, _ = self.solve_with_current_goals()
            data = {'base_transform':self.base_transform,'joint_states':self.displayed_state}
            rospy.logdebug('{0}'.format(data))
            self.result_pub.publish(String(data=json.dumps(data)))


    def solve_with_current_goals(self):
        inputs = []
        for idx,objective in enumerate(self.config.objectives):
            input_value = {}
            input_value.update({'weight':self.current_weights[idx]})
            input_value.update(self.current_directions[idx])
            inputs.append(ObjectiveInput(**input_value))
        return self.solver.solve(inputs,datetime.utcnow().timestamp(),max_retries=0,max_iterations=self.iterations)


def main():
    rospy.init_node('lively_ik_solver')
    node = SolverNode()
    while not rospy.is_shutdown():
        node.standard_loop()

if __name__ == '__main__':
    main()