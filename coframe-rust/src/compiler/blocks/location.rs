use std::collections::HashMap;
use serde::{Serialize, Deserialize};
use crate::compiler::properties::{Property};
use crate::compiler::utils::{DataType};

#[derive(Serialize,Deserialize,Clone,Debug)]
#[serde(rename_all = "camelCase")]
pub struct LocationBlock {
    pub id: String,
    pub name: String,
    pub data_type: DataType,
    pub description: String,
    pub can_delete: bool,
    pub can_edit: bool,
    pub editing: bool,
    pub selected: bool,
    pub properties: HashMap<String,Property>
}
/*
description: {
        name: 'Description',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        default: "",
        isList: false,
        fullWidth: true
      },
      position: {
        name: 'Position',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        default: {x: null, y: null, z: null},
        isList: false,
        fullWidth: true
      },
      rotation: {
        name: 'Rotation',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: {x: null, y: null, z: null, w: null},
        isList: false,
        fullWidth: true
      },
      states: {
        name: 'States',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: {}
      },
      reachability: {
        name: 'Reachability',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: {}
      },
      status: {
        name: 'Status',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: STATUS.PENDING
      },
      compileFn: {
        name: 'Compile Function',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: COMPILE_FUNCTIONS.POSE
      },
      compiled: {
        name: 'Compiled',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: {}
      },
      updateFields: {
        name: 'Update Fields',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: ['position','rotation']
      },
      singleton: {
        name: 'singleton',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: true
      }
*/