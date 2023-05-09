use lively::{
    objectives::{
        core::{
            base::{CollisionAvoidanceObjective, SmoothnessMacroObjective, LinkVelocityMinimizationObjective, JointVelocityMinimizationObjective},
            matching::{OrientationMatchObjective, PositionMatchObjective, PositionLineMatchObjective, JointMatchObjective},
        },
        objective::Objective,
    },
    utils::{
        goals::Goal,
        info::{ProximityInfo, ScalarRange, TransformInfo, Line},
        robot_model::RobotModel,
        shapes::Shape,
        state::State,
    },
};
use nalgebra::geometry::Isometry3;
use num_traits::{float::Float, ToPrimitive};
use rand::distributions::{Distribution, WeightedIndex};
use rand::rngs::ThreadRng;
use rand::{thread_rng, Rng};
use serde::{Deserialize, Serialize};
use serde_wasm_bindgen;
use std;
use std::collections::HashMap;
use std::fmt::Debug;
use std::ops::Sub;
use wasm_bindgen::prelude::*;
// use crate::compiler::compiled::Compiled;
// use crate::compiler::blocks::block::Block;
extern crate console_error_panic_hook;
// use serde::{Deserialize, Serialize};
// use wasm_bindgen::prelude::*;
// use lively_tk::lively_tk::Solver;
// use lively_tk::objectives::objective::*;
// use lively_tk::utils::goals::*;
// use lively_tk::utils::info::*;
// use lively_tk::utils::shapes::*;
// use lively_tk::utils::state::State;
// use std::collections::HashMap;
pub mod compiler;
pub use lively::Solver;

#[wasm_bindgen]
extern "C" {
    fn alert(s: &str);
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

macro_rules! console_log {
    // Note that this is using the `log` function imported above during
    // `bare_bones`
    ($($t:tt)*) => (log(&format_args!($($t)*).to_string()))
}

fn serialize<T>(obj: &T) -> Result<JsValue, serde_wasm_bindgen::Error>
where
    T: Serialize,
{
    Ok(obj.serialize(&serde_wasm_bindgen::Serializer::json_compatible())?)
}

#[wasm_bindgen]
pub fn big_computation() {
    alert("Big computation in Rust");
}

#[wasm_bindgen]
pub fn welcome(name: &str) -> String {
    return format!("Hello {}, from Rust!", name);
}

#[wasm_bindgen(js_name = planTrajectory)]
pub fn plan_trajectory_js(
    urdf: String,
    waypoints: JsValue,
    shapes: JsValue,
    root_bounds: JsValue,
    ik_link: String,
    is_ik: bool,
) -> Result<JsValue, serde_wasm_bindgen::Error> {
    // Convert inputs into required forms from JsValues
    let waypoint_joints: Vec<HashMap<String, f64>> =
        serde_wasm_bindgen::from_value(waypoints).unwrap_or(vec![]);
    let environment: Vec<Shape> = serde_wasm_bindgen::from_value(shapes).unwrap_or(vec![]);
    let bounds: Vec<ScalarRange> = serde_wasm_bindgen::from_value(root_bounds).unwrap_or(vec![
        ScalarRange::new(0.0, 0.0),
        ScalarRange::new(0.0, 0.0),
        ScalarRange::new(0.0, 0.0),
        ScalarRange::new(0.0, 0.0),
        ScalarRange::new(0.0, 0.0),
        ScalarRange::new(0.0, 0.0),
    ]);
    return serialize(&plan_trajectory(
        urdf,
        waypoint_joints,
        environment,
        bounds,
        ik_link,
        is_ik,
    ));
}

#[wasm_bindgen(js_name = computePose)]
pub fn compute_pose_js(
    urdf: String,
    goal: JsValue,
    origin: JsValue,
    attachment_link: String,
    shapes: JsValue,
) -> Result<JsValue, serde_wasm_bindgen::Error> {
    let goal_iso: Isometry3<f64> = serde_wasm_bindgen::from_value(goal).unwrap();
    let origin_iso: Isometry3<f64> = serde_wasm_bindgen::from_value(origin).unwrap();
    let shape_vec: Option<Vec<Shape>> = serde_wasm_bindgen::from_value(shapes).unwrap();
    return serialize(&compute_pose(urdf,goal_iso,origin_iso,attachment_link,shape_vec));
}

pub struct Planner {
    pub robot_model: RobotModel,
    pub upper_bounds: Vec<f64>,
    pub lower_bounds: Vec<f64>,
    pub default_x: Vec<f64>,
}

impl Planner {
    pub fn new(urdf: String, shapes: Vec<Shape>, root_bounds: Vec<ScalarRange>) -> Self {
        // Define the robot model
        let robot_model: RobotModel = RobotModel::new(urdf, shapes, &None, root_bounds.clone());
        let initial_frames = robot_model.get_default_state().frames;
        let proximity_info: Vec<ProximityInfo> = vec![];
        robot_model
            .collision_manager
            .lock()
            .unwrap()
            .compute_ground_truth_distance_hashmap(&initial_frames, &proximity_info);

        let mut upper_bounds: Vec<f64> = root_bounds
            .iter()
            .map(|bound| bound.value + bound.delta)
            .collect();
        let mut lower_bounds: Vec<f64> = root_bounds
            .iter()
            .map(|bound| bound.value - bound.delta)
            .collect();
        for joint in &robot_model.joints {
            // Only include non-mimic joints
            match joint.mimic {
                Some(_) => {}
                None => {
                    lower_bounds.push(joint.lower_bound.clone());
                    upper_bounds.push(joint.upper_bound.clone());
                }
            }
        }

        // Get the default x vec. This is useful for generating random starting points
        let default_x: Vec<f64> = robot_model.get_x(&robot_model.get_default_state()).clone();

        // Normalize the collision map
        let mut sampled_states: Vec<HashMap<String, TransformInfo>> = vec![];
        let mut rng: ThreadRng = thread_rng();

        for _ in 0..1000 {
            let mut x: Vec<f64> = default_x.clone();
            for i in 0..x.len() {
                let upper: f64 = upper_bounds[i].min(50.0);
                let lower: f64 = lower_bounds[i].max(-50.0);

                if upper - lower > 0.0 {
                    x[i] = rng.gen_range(lower..upper);
                }
            }
            // println!("x: {:?}",x);
            let state = robot_model.get_state(&x, false, 0.0);
            sampled_states.push(state.frames);
        }
        robot_model
            .collision_manager
            .lock()
            .unwrap()
            .compute_a_table(&sampled_states);

        return Self {
            robot_model,
            upper_bounds,
            lower_bounds,
            default_x,
        };
    }

    pub fn check_state(&self, vector: &Vec<f64>) -> bool {
        let state: State = self.robot_model.get_state(&vector, true, 0.0);
        return state_proximity(&state);
    }

    pub fn check_state_ik(&self, vector: &Vec<f64>, ik_link: &String, start: &Isometry3<f64>, goal: &Isometry3<f64>) -> bool {
        let state: State = self.robot_model.get_state(&vector, true, 0.0);

        let ik_link_iso3 = state.get_link_transform(ik_link);

        let mut passes: bool = true;

        for proximity in &state.proximity {
            if proximity.physical
                && proximity.distance < 0.001
                && proximity.average_distance.unwrap_or(1.0) > 0.001
            {
                passes = false;
                break;
            }
        }

        if passes {
            let ik_tolerance = 1.0;
            let ap = ik_link_iso3.translation.vector - start.translation.vector;
            let ab = goal.translation.vector - start.translation.vector;
            let projected = goal.translation.vector + (ab*(ap.dot(&ab)/ab.dot(&ab)));
            let x_val = (ik_link_iso3.translation.vector - projected).norm();

            if x_val > ik_tolerance {
                passes = false;
            }
        }

        return passes;
    }

    pub fn get_sample(&self) -> Vec<f64> {
        let mut rng_local: ThreadRng = thread_rng();
        let mut rand_x: Vec<f64> = self.default_x.clone();
        for i in 0..rand_x.len() {
            let upper: f64 = self.upper_bounds[i].min(50.0);
            let lower: f64 = self.lower_bounds[i].max(-50.0);

            if upper - lower > 0.0 {
                rand_x[i] = rng_local.gen_range(lower..upper);
            }
        }
        println!("rand {:?}", rand_x);
        return rand_x;
    }
}

pub fn plan_trajectory(
    urdf: String,
    waypoints: Vec<HashMap<String, f64>>,
    shapes: Vec<Shape>,
    root_bounds: Vec<ScalarRange>,
    ik_link: String,
    is_ik: bool,
) -> PlanningResult {
    // Define the robot model
    let planner = Planner::new(urdf.clone(), shapes.clone(), root_bounds.clone());

    let waypoint_pairs: Vec<(&HashMap<String, f64>, &HashMap<String, f64>)> = waypoints
        .iter()
        .zip(waypoints.iter().skip(1))
        .collect::<Vec<(&HashMap<String, f64>, &HashMap<String, f64>)>>();

    let mut trajectory: Vec<State> = vec![];
    let mut add_start_state = true;
    for (start_wp, end_wp) in &waypoint_pairs {
        // For each pair of waypoints, try to calculate a trajcectory between them.
        // If successful, append that list to the set of trajectory points
        let mut start_state = planner.robot_model.get_default_state();
        start_wp.iter().for_each(|(key, joint_value)| {
            start_state.joints.insert(key.clone(), joint_value.clone());
        });
        let mut end_state = planner.robot_model.get_default_state();
        end_wp.iter().for_each(|(key, joint_value)| {
            end_state.joints.insert(key.clone(), joint_value.clone());
        });
        let start_vec = planner.robot_model.get_x(&start_state);
        let end_vec = planner.robot_model.get_x(&end_state);
        let start_state = planner.robot_model.get_state(&start_vec.to_vec(), false, 0.0);
        let start_iso3 = start_state.get_link_transform(&ik_link);
        let end_state = planner.robot_model.get_state(&end_vec.to_vec(), false, 0.0);
        let end_iso3 = end_state.get_link_transform(&ik_link);

        if add_start_state {
            trajectory.push(start_state.clone());
            add_start_state = false;
        }

        println!("Start {:?}", start_vec);
        println!("End {:?}", end_vec);
        if is_ik {
            println!("I am running hte IK stuff and things");
            
            let mut objectives: HashMap<String, Objective> = HashMap::new();
            objectives.insert(
                "position_line_match".to_string(),
                Objective::PositionLineMatch(PositionLineMatchObjective::new(
                    "position_line_match".to_string(),
                    7.0,
                    ik_link.clone(),
                )),
            );
            objectives.insert(
                "position_match".to_string(),
                Objective::PositionMatch(PositionMatchObjective::new(
                    "position_match".to_string(),
                    4.0,
                    ik_link.clone(),
                )),
            );
            // objectives.insert(
            //     "smoothness".to_string(),
            //     Objective::SmoothnessMacro(SmoothnessMacroObjective::new(
            //         "smoothness".to_string(),
            //         7.0,
            //         true,
            //         false,
            //         true
            //     )),
            // );
            objectives.insert(
                "joint_velocity".to_string(),
                Objective::JointVelocityMinimization(JointVelocityMinimizationObjective::new(
                    "joint_velocity".to_string(),
                    5.0,
                )),
            );
            objectives.insert(
                "joint_velocity".to_string(),
                Objective::LinkVelocityMinimization(LinkVelocityMinimizationObjective::new(
                    "joint_velocity".to_string(),
                    7.0,
                )),
            );
            objectives.insert(
                "collision_avoidance".to_string(),
                Objective::CollisionAvoidance(CollisionAvoidanceObjective::new(
                    "collision_avoidance".to_string(),
                    1.0,
                )),
            );
            objectives.insert(
                "orientation_match".to_string(),
                Objective::OrientationMatch(OrientationMatchObjective::new(
                    "orientation_match".to_string(),
                    3.0,
                    ik_link.clone(),
                )),
            );
            for joint_name in &planner.robot_model.joint_names {
                objectives.insert(
                    joint_name.to_string(),
                    Objective::JointMatch(JointMatchObjective::new(
                        joint_name.to_string(),
                        0.0,
                        joint_name.to_string()
                    )),
                );
            }

            let mut goals: HashMap<String, Goal> = HashMap::new();
            goals.insert(
                "position_match".to_string(),
                Goal::Translation(end_iso3.translation)
            );
            goals.insert(
                "orientation_match".to_string(),
                Goal::Rotation(end_iso3.rotation)
            );
            goals.insert(
                "position_line_match".to_string(),
                Goal::Line(Line::new(start_iso3.translation.vector, end_iso3.translation.vector))
            );
            for joint_name in &planner.robot_model.joint_names {
                println!("Joint {} Position Value {}", joint_name.to_string(), end_state.get_joint_position(joint_name));
                goals.insert(
                    joint_name.to_string(),
                    Goal::Scalar(end_state.get_joint_position(joint_name))
                );
            }

            let mut solver = Solver::new(
                urdf.clone(),
                objectives,
                Some(root_bounds.clone()),
                Some(shapes.clone()),
                Some(start_state.clone()),
                Some(5),
                Some(75),
                None,
            );

            solver.compute_average_distance_table();
            
            let mut weights: HashMap<String, f64> = HashMap::new();

            let mut keep_looping = true;
            let mut failed = false;
            let mut time = 0.0;
            let mut previous_state = start_state.clone();
            let mut distance_between_start_and_end = 0.0;
            let mut non_change_counter = 0;
            let max_non_change = 10;
            let non_change_threshold = 0.01;

            // Calculate the distance between the start and end waypoints
            for joint_name in &planner.robot_model.joint_names {
                let e = end_state.get_joint_position(joint_name);
                let s = start_state.get_joint_position(joint_name);
                distance_between_start_and_end += (e - s) * (e - s);
            }
            distance_between_start_and_end = distance_between_start_and_end.sqrt();
            
            while keep_looping {
                // Calculate distance from current state to the end state
                let mut distance_to_end_wp = 0.0;
                for joint_name in &planner.robot_model.joint_names {
                    let e = end_state.get_joint_position(joint_name);
                    let p = previous_state.get_joint_position(joint_name);
                    distance_to_end_wp += (e - p) * (e - p);
                }
                distance_to_end_wp = distance_to_end_wp.sqrt();
                // Ratio of distances
                let percent_trajectory_completed = distance_to_end_wp / distance_between_start_and_end;

                // Update joint weights
                let new_weight = 10.0 / (percent_trajectory_completed / 10.0).exp();
                // let new_weight = -0.09 * percent_trajectory_completed + 20.0;
                println!("New weight {} Percept Completed {}", new_weight, percent_trajectory_completed);
                for joint_name in &planner.robot_model.joint_names {
                    weights.insert(
                        joint_name.to_string(),
                        new_weight
                    );
                }

                // Solve new state and update trajectory
                let new_state = solver.solve(goals.clone(), weights.clone(), time, None);
                trajectory.push(new_state.clone());

                // 
                let mut state_distance = 0.0;
                let mut end_distance = 0.0;
                for joint_name in &planner.robot_model.joint_names {
                    let p = previous_state.get_joint_position(joint_name);
                    let n = new_state.get_joint_position(joint_name);
                    let e = end_state.get_joint_position(joint_name);
                    state_distance += (n - p) * (n - p);
                    end_distance += (e - n) * (e - n);
                }
                
                // Check if whether a sufficient change has occurred
                if state_distance.sqrt() < non_change_threshold {
                    non_change_counter += 1;
                }

                // Check if we exceed the maximum amount of stalling
                if non_change_counter >= max_non_change {
                    failed = true;
                    keep_looping = false;
                }
                // TODO, reset counter when changes occur

                // Check if we've reached the end goal
                if end_distance.sqrt() < 0.01 {
                    keep_looping = false;
                    trajectory.push(end_state.clone());
                }

                previous_state = new_state;
                time = time + 0.1;
            }

            if failed {
                // Return the current trajectory as a failed planning result
                return PlanningResult::Failure { trajectory };
            }
        } else {
            let planning_result = rrt::dual_rrt_connect(
                &start_vec.as_slice(),
                &end_vec.as_slice(),
                |x| planner.check_state(&x.to_vec()),
                || planner.get_sample(),
                0.1,
                1000,
            );
            println!("Computed Planning Result");

            match planning_result {
                Ok(p) => {
                    let smoothed_path =
                        filter(&p, |x| planner.check_state(&x.to_vec()), p.len() * 200, 0).unwrap();
                    // let mut raw_path = p.clone();
                    // // Smooth the result
                    // rrt::smooth_path(
                    //     &mut raw_path,
                    //     |x| planner.check_state(&x.to_vec()),
                    //     0.1,
                    //     1,
                    // );
                    smoothed_path.iter().for_each(|x| {
                        trajectory.push(planner.robot_model.get_state(x, true, 0.0));
                    })
                }
                _ => {
                    // Return the current trajectory as a failed planning result
                    return PlanningResult::Failure { trajectory };
                }
            }
        }
        println!("Smoothed Result");
    }

    return PlanningResult::Success { trajectory };
}

pub fn compute_pose(
    urdf: String,
    goal: Isometry3<f64>,
    origin: Isometry3<f64>,
    attachment_link: String,
    shapes: Option<Vec<Shape>>,
) -> PoseResult {
    let origin_euler = origin.rotation.euler_angles();
    let root_bounds: Vec<ScalarRange> = vec![
        ScalarRange::new(origin.translation.x, 0.0),
        ScalarRange::new(origin.translation.y, 0.0),
        ScalarRange::new(origin.translation.z, 0.0),
        ScalarRange::new(origin_euler.0, 0.0),
        ScalarRange::new(origin_euler.1, 0.0),
        ScalarRange::new(origin_euler.2, 0.0),
    ];
    let mut objectives: HashMap<String, Objective> = HashMap::new();
    objectives.insert(
        "position_match".to_string(),
        Objective::PositionMatch(PositionMatchObjective::new(
            "position_match".to_string(),
            50.0,
            attachment_link.clone(),
        )),
    );
    objectives.insert(
        "orientation_match".to_string(),
        Objective::OrientationMatch(OrientationMatchObjective::new(
            "orientation_match".to_string(),
            30.0,
            attachment_link.clone(),
        )),
    );
    objectives.insert(
        "collision_avoidance".to_string(),
        Objective::CollisionAvoidance(CollisionAvoidanceObjective::new(
            "collision_avoidance".to_string(),
            0.1,
        )),
    );

    let mut solver = Solver::new(
        urdf,
        objectives,
        Some(root_bounds),
        shapes,
        None,
        Some(5),
        Some(75),
        None,
    );
    solver.compute_average_distance_table();
    let jacobian_result = solver.jacobian_ik(goal, origin, attachment_link.clone());
    match jacobian_result {
        Ok(state) => {

            // TODO: run check_state
            if state_proximity(&state) {
                return PoseResult::Success { state }
            }
            solver.reset(state, HashMap::new());
            // TODO: if fail, basically do the err approach
            return normal_solve(solver, goal, attachment_link);
        },
        Err(_errmsg) => {
            return normal_solve(solver, goal, attachment_link);
        }
    }
}

pub fn state_proximity(state: &State) -> bool {
    let mut passes: bool = true;

    for proximity in &state.proximity {
        if proximity.physical
            && proximity.distance < 0.001
            && proximity.average_distance.unwrap_or(1.0) > 0.001
        {
            passes = false;
            break;
        }
    }

    return passes;
}

pub fn normal_solve(
    mut solver: Solver,
    goal: Isometry3<f64>,
    attachment_link: String
) -> PoseResult {
    let mut goals: HashMap<String, Goal> = HashMap::new();
    goals.insert(
        "position_match".to_string(),
        Goal::Translation(goal.translation),
    );
    goals.insert(
        "orientation_match".to_string(),
        Goal::Rotation(goal.rotation),
    );
    let mut result: PoseResult = PoseResult::Failure {
        state: solver.get_current_state(),
        message: "Could not resolve after 0 iterations.".to_string(),
    };
    for i in 0..20 {
        let candidate_state = solver.solve(goals.clone(), HashMap::new(), 0.0, None);
        let position_diff: f64 = (candidate_state
            .get_link_transform(&attachment_link)
            .translation
            .vector
            - goal.translation.vector)
            .norm();
        let rotation_diff: f64 = candidate_state
            .get_link_transform(&attachment_link)
            .rotation
            .angle_to(&goal.rotation);
        if position_diff <= 0.01 && rotation_diff <= 0.01 {
            result = PoseResult::Success {
                state: candidate_state,
            };
            break;
        } else {
            result = PoseResult::Failure {
                state: candidate_state,
                message: format!("Could not resolve after {} iterations.", i),
            }
        }
    }

    return result;
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(tag = "code")]
pub enum PlanningResult {
    Success { trajectory: Vec<State> },
    Failure { trajectory: Vec<State> },
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(tag = "code")]
pub enum PoseResult {
    Success { state: State },
    Failure { state: State, message: String },
}

#[cfg(test)]
mod tests {
    use crate::{compute_diff, plan_trajectory, PlanningResult};
    use lively::utils::info::ScalarRange;
    use std::collections::HashMap;
    use std::fs;

    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }

    #[test]
    fn test_trajectory_planning() {
        let urdf =
            fs::read_to_string("./src/ur5e.urdf").expect("Something went wrong reading the file");

        let mut wp1: HashMap<String, f64> = HashMap::new();
        wp1.insert("shoulder_lift_joint".into(), -1.1413);
        wp1.insert("shoulder_pan_joint".into(), 1.3676);
        wp1.insert("elbow_joint".into(), -2.4042);
        wp1.insert("wrist_1_joint".into(), -0.2246);
        wp1.insert("wrist_2_joint".into(), -4.638);
        wp1.insert("wrist_3_joint".into(), 6.1430);

        let mut wp2: HashMap<String, f64> = HashMap::new();
        wp2.insert("shoulder_lift_joint".into(), -0.95060);
        wp2.insert("shoulder_pan_joint".into(), 1.99524);
        wp2.insert("elbow_joint".into(), -2.395939);
        wp2.insert("wrist_1_joint".into(), -0.16186);
        wp2.insert("wrist_2_joint".into(), -4.56309);
        wp2.insert("wrist_3_joint".into(), 6.17025);

        let mut wp3: HashMap<String, f64> = HashMap::new();
        wp3.insert("shoulder_lift_joint".into(), -1.9471);
        wp3.insert("shoulder_pan_joint".into(), -3.04705);
        wp3.insert("elbow_joint".into(), -2.2490);
        wp3.insert("wrist_1_joint".into(), 3.50395);
        wp3.insert("wrist_2_joint".into(), -1.539);
        wp3.insert("wrist_3_joint".into(), -1.5284);

        let root_bounds: Vec<ScalarRange> = vec![
            ScalarRange::new(0.0, 0.0),
            ScalarRange::new(-0.15, 0.0),
            ScalarRange::new(0.0, 0.0),
            ScalarRange::new(0.0, 0.0),
            ScalarRange::new(0.0, 0.0),
            ScalarRange::new(0.0, 0.0),
        ];

        let wps = vec![wp1, wp2, wp3];
        let length = wps.len();

        let plan_result: PlanningResult = plan_trajectory(urdf, wps, vec![], root_bounds, "tool0".to_string(),  false);
        println!("{:?}", plan_result);
        match plan_result {
            PlanningResult::Failure {
                trajectory: _trajectory,
            } => assert!(false),
            PlanningResult::Success { trajectory } => assert!(trajectory.len() > length + 1),
        }
        // assert_matches!()
    }


    #[test]
    fn test_trajectory_planning_ik() {
        let urdf =
            fs::read_to_string("./src/ur5e.urdf").expect("Something went wrong reading the file");

        let mut wp1: HashMap<String, f64> = HashMap::new();
        wp1.insert("shoulder_lift_joint".into(), -1.4344370712156858);
        wp1.insert("shoulder_pan_joint".into(), 1.4803176226228232);
        wp1.insert("elbow_joint".into(), 1.9321455633363789);
        wp1.insert("wrist_1_joint".into(), 2.6425606393623737);
        wp1.insert("wrist_2_joint".into(), -1.478588627652706);
        wp1.insert("wrist_3_joint".into(), 0.00020121298222051235);

        let mut wp2: HashMap<String, f64> = HashMap::new();
        wp2.insert("shoulder_lift_joint".into(), -1.247770144502285);
        wp2.insert("shoulder_pan_joint".into(), 0.6712059012773973);
        wp2.insert("elbow_joint".into(), 1.690894776746486);
        wp2.insert("wrist_1_joint".into(), 2.696739679727712);
        wp2.insert("wrist_2_joint".into(), -0.6706905188057736);
        wp2.insert("wrist_3_joint".into(), 0.001609596542650402);

        let root_bounds: Vec<ScalarRange> = vec![
            ScalarRange::new(0.0, 0.0),
            ScalarRange::new(-0.15, 0.0),
            ScalarRange::new(0.0, 0.0),
            ScalarRange::new(0.0, 0.0),
            ScalarRange::new(0.0, 0.0),
            ScalarRange::new(0.0, 0.0),
        ];

        let wps = vec![wp1, wp2];
        let length = wps.len();

        let plan_result: PlanningResult = plan_trajectory(urdf, wps, vec![], root_bounds, "tool0".to_string(),  true);
        println!("{:?}", plan_result);
        match plan_result {
            PlanningResult::Failure {
                trajectory: _trajectory,
            } => assert!(false),
            PlanningResult::Success { trajectory } => assert!(trajectory.len() > length + 1),
        }
        // assert_matches!()
    }

    #[test]
    fn test_compute_difference() {
        let input_vec = vec![
            vec![0.0, 0.1, 0.4],
            vec![0.0, 0.2, 0.2],
            vec![0.0, 0.3, 0.3],
            vec![0.0, 0.4, 0.1],
        ];
        let output_vec = compute_diff(input_vec);
        let expected_vec = vec![
            vec![0.0, 0.1, -0.2],
            vec![0.0, 0.1, 0.1],
            vec![0.0, 0.1, -0.2],
        ];
        for (i, expected_frame) in expected_vec.iter().enumerate() {
            assert_ne!(output_vec.get(i), None);
            let output_frame = output_vec.get(i).unwrap();
            for (j, expected_value) in expected_frame.iter().enumerate() {
                assert_ne!(output_frame.get(j), None);
                // Check that the values are similar within some epsilon
                assert!(expected_value - output_frame.get(j).unwrap() < 0.0001)
            }
        }
    }

    #[test]
    fn test_compute_vec_norm() {
        let input_vec = vec![
            vec![0.0, 0.1, 0.4],
            vec![0.0, 0.2, 0.2],
            vec![0.0, 0.3, 0.3],
            vec![0.0, 0.4, 0.1],
        ];
        let output_vec = compute_diff(input_vec);
        let expected_vec = vec![
            vec![0.0, 0.1, -0.2],
            vec![0.0, 0.1, 0.1],
            vec![0.0, 0.1, -0.2],
        ];
        for (i, expected_frame) in expected_vec.iter().enumerate() {
            assert_ne!(output_vec.get(i), None);
            let output_frame = output_vec.get(i).unwrap();
            for (j, expected_value) in expected_frame.iter().enumerate() {
                assert_ne!(output_frame.get(j), None);
                // Check that the values are similar within some epsilon
                assert!(expected_value - output_frame.get(j).unwrap() < 0.0001)
            }
        }
    }
}

pub fn filter<FF, N>(
    state_vector: &Vec<Vec<N>>,
    mut is_free: FF,
    rounds: usize,
    _window_size: usize,
) -> Result<Vec<Vec<N>>, String>
where
    FF: FnMut(&[N]) -> bool,
    N: Float
        + Debug
        + Sub<N>
        + std::iter::Sum
        + Default
        + rand::distributions::uniform::SampleUniform
        + for<'a> std::ops::AddAssign<&'a N>
        + ToPrimitive,
{
    let mut filtered_vector = state_vector.clone();
    let mut rng = thread_rng();

    for _i in 0..rounds {
        // Calculate the second derivative to find places wher ethe rates are changing quickly
        let mut weights = vec_norm(compute_diff(compute_diff(filtered_vector.clone())));
        weights.insert(0, N::zero());
        weights.push(N::zero());

        // Use the derivative to create a weighted distribution to sample points for improvement
        let dist: WeightedIndex<N> = WeightedIndex::new(&weights).unwrap();
        let candidate_idx = dist.sample(&mut rng);

        let neighbor_1: Vec<N> = filtered_vector[candidate_idx - 1].clone();
        let neighbor_2: Vec<N> = filtered_vector[candidate_idx + 1].clone();
        let candidate: Vec<N> = filtered_vector[candidate_idx].clone();

        let candidate_replacement: Vec<N> = candidate
            .iter()
            .enumerate()
            .map(|(element_idx, element)| {
                ((*neighbor_1.get(element_idx).unwrap_or(element)
                    + *neighbor_2.get(element_idx).unwrap_or(element))
                    / N::from(2.0).unwrap())
                    * N::from(0.25).unwrap()
                    + *element * N::from(0.75).unwrap()
            })
            .collect();
        if is_free(candidate_replacement.as_slice()) {
            filtered_vector[candidate_idx] = candidate_replacement
        }
    }

    Ok(filtered_vector)
}

pub fn compute_diff<N>(state_vector: Vec<Vec<N>>) -> Vec<Vec<N>>
where
    N: Float + Debug + Sub<N>,
{
    return state_vector
        .iter()
        .zip(state_vector.iter().skip(1))
        .collect::<Vec<(&Vec<N>, &Vec<N>)>>()
        .iter()
        .map(|(start, end)| {
            start
                .iter()
                .enumerate()
                .map(|(i, v)| *end.get(i).unwrap_or(v) - *v)
                .collect()
        })
        .collect();
}

pub fn vec_norm<N>(state_vector: Vec<Vec<N>>) -> Vec<N>
where
    N: Float + Debug + std::iter::Sum,
{
    return state_vector
        .iter()
        .map(|state| state.iter().map(|v| v.powi(2) as N).sum::<N>().sqrt())
        .collect();
}
