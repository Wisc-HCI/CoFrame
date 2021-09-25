
import json

from lively_tk import ObjectiveInput, Solver, parse_config_data
from datetime import datetime
from geometry_msgs.msg import Pose


class LivelyTKSolver(object):
    
    def __init__(self, config_file_path, return_frames=True):
        self._returnFrames = return_frames

        # Load config file
        with open(config_file_path) as f:
            self.config_data = json.load(f)

        self.config = parse_config_data(self.config_data)
        self.solver = Solver(self.config)

        self.iterations = 15*(3+len(self.config_data['starting_config'][1]))
        self.base_transform = self.config_data['starting_config'][0]
        self.current_weights = self.config.default_weights

        # Parse the rust goal specs into a python dictionary
        _directions = []
        for default_goal in self.config.default_goals:
            goal_data = {}
            for field in ['scalar','vector','quaternion']:
                value = getattr(default_goal,field)
                if value:
                    goal_data[field] = value
            _directions.append(goal_data)

        self.target_directions = _directions

    def reset(self):
        self.solver.reset(
            self.config_data['starting_config'][0],
            self.config_data['starting_config'][1])
        return self.config_data['starting_config'][1], self.config_data["joint_ordering"]

    def set_joints(self, joints):
        self.solver.reset(
            self.config_data['starting_config'][0],
            joints)

    def step(self, pose):
        if self.solver == None:
            raise Exception('Solver not initialized')

        self.target_directions[0] = {'vector':[pose.position.x, pose.position.y, pose.position.z]}
        self.target_directions[1] = {'quaternion':[pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]}

        inputs = []
        for idx, objective in enumerate(self.config.objectives):
            input_value = {}
            input_value.update({'weight':self.current_weights[idx]})
            input_value.update(self.target_directions[idx])
            inputs.append(ObjectiveInput(**input_value))

        jNames = self.config_data["joint_ordering"]
        fNames = self.config_data["joint_names"][0] + [self.config_data["ee_fixed_joints"][0]]
        
        base_tf, joints, frames = self.solver.solve(
            inputs,
            datetime.utcnow().timestamp(),
            max_retries=0,
            max_iterations=self.iterations,
            only_core=True,
            return_frames=self._returnFrames)

        return (joints, jNames), (frames[0], fNames)

    @property
    def joint_names(self):
        return self.config_data["joint_ordering"]

    @property
    def frame_names(self):
        return self.config_data["joint_names"][0] + [self.config_data["ee_fixed_joints"][0]]

    @classmethod
    def get_ee_pose(cls, frames):
        pose = Pose()

        pos, rot = frames[-1]

        (x,y,z) = pos
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        (w,x,y,z) = rot
        pose.orientation.w = w
        pose.orientation.x = x
        pose.orientation.y = y
        pose.orientation.z = z

        return pose