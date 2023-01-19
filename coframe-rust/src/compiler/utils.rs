use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use lively::utils::info::ProximityInfo;

pub fn set_panic_hook() {
    // When the `console_error_panic_hook` feature is enabled, we can call the
    // `set_panic_hook` function at least once during initialization, and then
    // we will get better error messages if our code ever panics.
    //
    // For more details see
    // https://github.com/rustwasm/console_error_panic_hook#readme
    #[cfg(feature = "console_error_panic_hook")]
    console_error_panic_hook::set_once();
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct SimplePosition {
    x: f64,
    y: f64,
    z: f64
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct SimpleRotation {
    w: f64,
    x: f64,
    y: f64,
    z: f64
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct SimpleScale {
    x: f64,
    y: f64,
    z: f64
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct SimpleTransform {
    position: SimplePosition,
    rotation: SimpleRotation
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct SimpleFrame {
    position: SimplePosition,
    rotation: SimpleRotation,
    scale: SimpleScale
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct RobotState {
    joints: HashMap<String,f64>,
    links: HashMap<String,SimpleFrame>,
    proximity: Vec<ProximityInfo>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum DataType {
    INSTANCE,
    REFERENCE,
    CALL,
    ARGUMENT
}