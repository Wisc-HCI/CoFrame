use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use crate::compiler::compiled::Compiled;
use crate::compiler::status::Status;
use crate::compiler::utils::{SimplePosition,SimpleRotation,SimpleTransform,RobotState};

#[derive(Serialize, Deserialize,Clone,Debug)]
#[serde(untagged)]
pub enum Property {
    Status(Status),
    Number(f64),
    Text(String),
    TextArray(Vec<String>),
    Bool(bool),
    Position(SimplePosition),
    Rotation(SimpleRotation),
    Transform(SimpleTransform),
    RobotStates(HashMap<String,HashMap<String,RobotState>>),
    Reachability(HashMap<String,HashMap<String,bool>>),
    PropertyLookup(HashMap<String,Property>)
}