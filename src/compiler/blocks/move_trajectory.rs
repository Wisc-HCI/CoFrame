use std::collections::HashMap;

pub enum MotionType {
    IK,
    JOINT
}

pub struct TrajectoryBlock {
    pub name: String,
    pub description: String,
    pub trajectory: String,
    pub velocity: f64,
    pub motion_type: MotionType,
    pub compiled: HashMap<String,String>
}