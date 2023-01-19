use wasm_bindgen::prelude::*;
// use std::collections::HashMap;
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

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}

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

#[wasm_bindgen]
pub fn big_computation() {
    alert("Big computation in Rust");
}

#[wasm_bindgen]
pub fn welcome(name: &str) -> String {
    return format!("Hello {}, from Rust!", name);
}

// #[wasm_bindgen]
// pub fn compile(programData: &JsValue) -> JsValue {
//     let parsedProgramData = programData.into_serde().unwrap();
//     let mut memo: HashMap<String,Compiled> = HashMap::new();
//     return JsValue::from_serde(&memo).unwrap();
// }

// #[wasm_bindgen]
// pub fn compile(poses:JsValue) -> JsValue {
//     let blocks: Vec<Block> = poses.into_serde().unwrap_or(Vec::new());
//     return JsValue::from_serde(&blocks).unwrap()
// }

// Below is a re-export of LivelyTK bindings

// #[derive(Serialize,Deserialize,Clone,Debug)]
// pub struct ScalarRange {
//     value: f64,
//     delta: f64
// }

// #[wasm_bindgen(js_name=Solver)]
// pub struct JsSolver(Solver);

// #[wasm_bindgen(js_class=Solver)]
// impl JsSolver {
//     #[wasm_bindgen(constructor)]
//     pub fn new(
//         urdf: String, 
//         objectives: &JsValue, 
//         root_bounds: &JsValue,
//         shapes: &JsValue,
//         initial_state: &JsValue,
//         only_core: Option<bool>,
//         max_retries: Option<usize>,
//         max_iterations: Option<usize>,
//         collision_settings: &JsValue
//     ) -> Self {
//             console_error_panic_hook::set_once();
//             let inner_objectives:HashMap<String,Objective> = objectives.into_serde().unwrap();
//             let temp_bounds:Option<Vec<ScalarRange>> = root_bounds.into_serde().unwrap();
//             let inner_bounds:Option<Vec<(f64,f64)>> = temp_bounds.map(|bs| bs.iter().map(|b| (b.value,b.delta)).collect());
//             let inner_shapes:Option<Vec<Shape>> = shapes.into_serde().unwrap();
//             let inner_state:Option<State> = initial_state.into_serde().unwrap();
//             let inner_collision_settings:Option<CollisionSettingInfo> = collision_settings.into_serde().unwrap();
//             // let inner_retries: Option<u64> = max_retries.into_serde().unwrap();
//             // let inner_iterations: Option<usize> = max_iterations.into_serde().unwrap();
//             // let inner_core: Option<bool> = only_core.into_serde().unwrap();
//             Self(Solver::new(urdf, inner_objectives, inner_bounds, inner_shapes, inner_state, only_core, max_retries, max_iterations, inner_collision_settings))
//     }

//     #[wasm_bindgen(getter)]
//     pub fn objectives(&self) -> JsValue {
//         JsValue::from_serde(&self.0.objective_set.objectives).unwrap()
//     }

//     #[wasm_bindgen(setter)]
//     pub fn set_objectives(&mut self, objectives: JsValue) {
//         let inner_objectives: HashMap<String,Objective> = objectives.into_serde().unwrap();
//         self.0.set_objectives(inner_objectives);
//     }

//     #[wasm_bindgen(getter = currentState)]
//     pub fn current_state(&self) -> JsValue {
//         JsValue::from_serde(&self.0.get_current_state()).unwrap()
//     }

//     #[wasm_bindgen(getter = currentGoals)]
//     pub fn current_goals(&self) -> JsValue {
//         let mut goals: HashMap<String,Option<Goal>> = HashMap::new();
//         for (k,v) in self.0.objective_set.objectives.iter() {
//             goals.insert(k.clone(),v.get_goal());
//         }
//         JsValue::from_serde(&goals).unwrap()
//     }

//     #[wasm_bindgen(getter)]
//     pub fn links(&self) -> JsValue {
//         JsValue::from_serde(&self.0.robot_model.links).unwrap()
//     }

//     #[wasm_bindgen(getter)]
//     pub fn joints(&self) -> JsValue {
//         JsValue::from_serde(&self.0.robot_model.joints).unwrap()
//     }

//     pub fn reset(
//         &mut self, 
//         state: &JsValue,
//         weights: &JsValue,
//     ) {
//         let inner_state:State = state.into_serde().unwrap();
//         let inner_weights:HashMap<String,f64> = weights.into_serde().unwrap();
//         self.0.reset(inner_state,inner_weights);
//     }

//     pub fn solve(
//         &mut self,
//         goals: &JsValue,
//         weights: &JsValue,
//         time: f64,
//         shape_updates: &JsValue
//     ) -> JsValue {
//         let inner_goals: HashMap<String,Goal> = goals.into_serde().unwrap();
//         let inner_weights:HashMap<String,f64> = weights.into_serde().unwrap();
//         let inner_updates: Option<Vec<ShapeUpdate>> = shape_updates.into_serde().unwrap();
//         let state:State = self.0.solve(inner_goals,inner_weights,time,inner_updates);
//         return JsValue::from_serde(&state).unwrap();
//     }

//     #[wasm_bindgen(js_name = computeAverageDistanceTable)]
//     pub fn compute_average_distance_table(&mut self) -> JsValue {
//         return JsValue::from_serde(&self.0.compute_average_distance_table()).unwrap()
//     }
// }

// #[wasm_bindgen]
// pub fn solve(solver: &mut JsSolver, goals: &JsValue, weights: &JsValue, time: f64, shape_updates: &JsValue) -> JsValue {
//     let inner_goals: HashMap<String,Goal> = goals.into_serde().unwrap();
//     let inner_weights:HashMap<String,f64> = weights.into_serde().unwrap();
//     let inner_updates: Option<Vec<ShapeUpdate>> = shape_updates.into_serde().unwrap();
//     // console_log!("Received Goals: {:?}",inner_goals);
//     // console_log!("Received Weights: {:?}",inner_weights);
//     // console_log!("Received Updates: {:?}",inner_updates);
//     let state:State = solver.0.solve(inner_goals,inner_weights,time,inner_updates);
//     // console_log!("Produced State: {:?}",state);
//     return JsValue::from_serde(&state).unwrap();
// }

// #[wasm_bindgen]
// pub fn reset(solver: &mut JsSolver, state: &JsValue, weights: &JsValue) {
//     let inner_state:State = state.into_serde().unwrap();
//     let inner_weights:HashMap<String,f64> = weights.into_serde().unwrap();
//     solver.0.reset(inner_state,inner_weights);
// }
