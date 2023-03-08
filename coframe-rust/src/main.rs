use coframe_rust::{PlanningResult, plan_trajectory, filter};
use lively::utils::info::ScalarRange;
use std::{env,fs};
use std::collections::HashMap;

fn main() {
    let path = env::current_dir();
    println!("The current directory is {:?}", path);
    let urdf = fs::read_to_string("./src/ur5e.urdf").expect("Something went wrong reading the file");

    let mut wp1:HashMap<String, f64> = HashMap::new();
    wp1.insert("shoulder_lift_joint".into(),-1.1413);
    wp1.insert("shoulder_pan_joint".into(),1.3676);
    wp1.insert("elbow_joint".into(),-2.4042);
    wp1.insert("wrist_1_joint".into(),-0.2246);
    wp1.insert("wrist_2_joint".into(),-4.638);
    wp1.insert("wrist_3_joint".into(),6.1430);

    let mut wp2:HashMap<String, f64> = HashMap::new();
    wp2.insert("shoulder_lift_joint".into(),-0.95060);
    wp2.insert("shoulder_pan_joint".into(),1.99524);
    wp2.insert("elbow_joint".into(),-2.395939);
    wp2.insert("wrist_1_joint".into(),-0.16186);
    wp2.insert("wrist_2_joint".into(),-4.56309);
    wp2.insert("wrist_3_joint".into(),6.17025);

    let mut wp3:HashMap<String, f64> = HashMap::new();
    wp3.insert("shoulder_lift_joint".into(),-1.9471);
    wp3.insert("shoulder_pan_joint".into(),-3.04705);
    wp3.insert("elbow_joint".into(),-2.2490);
    wp3.insert("wrist_1_joint".into(),3.50395);
    wp3.insert("wrist_2_joint".into(),-1.539);
    wp3.insert("wrist_3_joint".into(),-1.5284);

    let root_bounds:Vec<ScalarRange> = vec![
        ScalarRange::new(0.0, 0.0),
        ScalarRange::new(-0.15, 0.0),
        ScalarRange::new(0.0, 0.0),
        ScalarRange::new(0.0, 0.0),
        ScalarRange::new(0.0, 0.0),
        ScalarRange::new(0.0, 0.0)
    ];

    let plan_result: PlanningResult = plan_trajectory(urdf, vec![wp1,wp2,wp3], vec![], root_bounds, false);
    // println!("{:?}",plan_result);
    match plan_result {
        PlanningResult::Failure{trajectory:_trajectory} => assert!(false),
        PlanningResult::Success{trajectory} => {
            println!("Success! Trajectory Length {}",trajectory.len());
            assert_ne!(trajectory.len(),0);
        },
    }
    
    let input_vec = vec![
            vec![0.0, 0.1, 0.4],
            vec![0.0, 0.2, 0.2],
            vec![0.0, 0.3, 0.3],
            vec![0.0, 0.4, 0.1],
        ];

    let result = filter(&input_vec,|_v| true,100,0);
    match result {
        Ok(smoothed) => {
            println!("Input\n{:?}",input_vec);
            println!("Smoothed\n{:?}",smoothed);
        },
        _ => {
            println!("No Result")
        }
    }

}