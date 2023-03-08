use lively::utils::{
    info::{ScalarRange, TransformInfo, ProximityInfo},
    robot_model::RobotModel,
    shapes::Shape,
    state::State,
};
use rand::rngs::ThreadRng;
use rand::{thread_rng, Rng};
use rand::distributions::{WeightedIndex,Distribution};
use serde::{Deserialize, Serialize};
use serde_wasm_bindgen;
use std;
use std::collections::HashMap;
use wasm_bindgen::prelude::*;
use std::ops::Sub;
use num_traits::{float::Float, ToPrimitive};
use std::fmt::Debug;
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
        is_ik,
    ));
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
        robot_model.collision_manager.lock().unwrap().compute_ground_truth_distance_hashmap(&initial_frames, &proximity_info);

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

        return Self {robot_model,upper_bounds,lower_bounds,default_x}

    }

    pub fn check_state(&self,vector: &Vec<f64>) -> bool {
        let state: State = self.robot_model.get_state(&vector, true, 0.0);
    
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
        println!("rand {:?}",rand_x);
        return rand_x
    }
}

pub fn plan_trajectory(
    urdf: String,
    waypoints: Vec<HashMap<String, f64>>,
    shapes: Vec<Shape>,
    root_bounds: Vec<ScalarRange>,
    _is_ik: bool,
) -> PlanningResult {
    // Define the robot model
    let planner = Planner::new(urdf,shapes,root_bounds);

    let waypoint_pairs: Vec<(&HashMap<String, f64>, &HashMap<String, f64>)> = waypoints
        .iter()
        .zip(waypoints.iter().skip(1))
        .collect::<Vec<(&HashMap<String, f64>, &HashMap<String, f64>)>>();

    let mut trajectory: Vec<State> = vec![];
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

        println!("Start {:?}", start_vec);
        println!("End {:?}", end_vec);

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
                let smoothed_path = filter(
                    &p,
                    |x| planner.check_state(&x.to_vec()),
                    p.len()*200, 
                    0
                ).unwrap();
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
        println!("Smoothed Result");
    }

    return PlanningResult::Success { trajectory };
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(tag = "code")]
pub enum PlanningResult {
    Success { trajectory: Vec<State> },
    Failure { trajectory: Vec<State> },
}

#[cfg(test)]
mod tests {
    use crate::{plan_trajectory, PlanningResult, compute_diff};
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

        let plan_result: PlanningResult = plan_trajectory(urdf, wps, vec![], root_bounds, false);
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
            vec![0.0, 0.1, -0.2]
        ];
        for (i,expected_frame) in expected_vec.iter().enumerate() {
            assert_ne!(output_vec.get(i),None);
            let output_frame = output_vec.get(i).unwrap();
            for (j, expected_value) in expected_frame.iter().enumerate() {
                assert_ne!(output_frame.get(j),None);
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
            vec![0.0, 0.1, -0.2]
        ];
        for (i,expected_frame) in expected_vec.iter().enumerate() {
            assert_ne!(output_vec.get(i),None);
            let output_frame = output_vec.get(i).unwrap();
            for (j, expected_value) in expected_frame.iter().enumerate() {
                assert_ne!(output_frame.get(j),None);
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
    N: Float + Debug + Sub<N> + std::iter::Sum + Default + rand::distributions::uniform::SampleUniform + for<'a> std::ops::AddAssign<&'a N> + ToPrimitive
{
    let mut filtered_vector = state_vector.clone();
    let mut rng = thread_rng();

    for _i in 0..rounds {
        // Calculate the second derivative to find places wher ethe rates are changing quickly
        let mut weights = vec_norm(compute_diff(compute_diff(filtered_vector.clone())));
        weights.insert(0,N::zero());
        weights.push(N::zero());

        // Use the derivative to create a weighted distribution to sample points for improvement
        let dist: WeightedIndex<N> = WeightedIndex::new(&weights).unwrap();
        let candidate_idx = dist.sample(&mut rng);

        let neighbor_1:Vec<N> = filtered_vector[candidate_idx-1].clone();
        let neighbor_2:Vec<N> = filtered_vector[candidate_idx+1].clone();
        let candidate:Vec<N> = filtered_vector[candidate_idx].clone();

        let candidate_replacement:Vec<N> = candidate
            .iter()
            .enumerate()
            .map(
                |(element_idx, element)| 
                    (
                        (
                            *neighbor_1.get(element_idx).unwrap_or(element) + 
                            *neighbor_2.get(element_idx).unwrap_or(element)
                        ) 
                    / N::from(2.0).unwrap()) 
                    * N::from(0.25).unwrap() + *element * N::from(0.75).unwrap()
            )
            .collect();
        if is_free(candidate_replacement.as_slice()) {
            filtered_vector[candidate_idx] = candidate_replacement
        }

    }
    
    Ok(filtered_vector)
}

pub fn compute_diff<N>(
    state_vector:Vec<Vec<N>>
) -> Vec<Vec<N>> 
where 
    N: Float + Debug + Sub<N> 
{
    return state_vector
        .iter()
        .zip(state_vector.iter().skip(1))
        .collect::<Vec<(&Vec<N>, &Vec<N>)>>()
        .iter().map(|(start,end)|{
            start.iter().enumerate().map(|(i,v)| *end.get(i).unwrap_or(v) - *v).collect()
        })
        .collect();
}

pub fn vec_norm<N>(state_vector:Vec<Vec<N>>) -> Vec<N> where N: Float + Debug + std::iter::Sum {
    return state_vector
        .iter()
        .map(|state| 
            state
                .iter()
                .map(
                    |v| v.powi(2) as N
                )
                .sum::<N>()
                .sqrt()
        )
        .collect();
}