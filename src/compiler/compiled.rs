use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use crate::compiler::status::Status;
use crate::compiler::steps::Step;
use crate::compiler::properties::Property;

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(rename_all = "camelCase")]
pub struct Compiled {
    #[serde(rename = "break")]
    pub should_break: bool,
    pub status: Status,
    pub steps: Option<Vec<Step>>,
    pub goal_pose: Option<Vec<Step>>,
    pub other_property_updates: Option<HashMap<String,Property>>,
    
}