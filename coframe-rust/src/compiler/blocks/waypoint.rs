use std::collections::HashMap;
use serde::{Serialize, Deserialize};
use crate::compiler::properties::{Property};
use crate::compiler::utils::{DataType};

#[derive(Serialize,Deserialize,Clone,Debug)]
#[serde(rename_all = "camelCase")]
pub struct WaypointBlock {
    pub id: String,
    pub name: String,
    pub data_type: DataType,
    pub description: String,
    pub can_delete: bool,
    pub can_edit: bool,
    pub editing: bool,
    pub selected: bool,
    pub properties: HashMap<String,Property>,
}