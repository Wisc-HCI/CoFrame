use wasm_bindgen::prelude::*;
// use std::collections::HashMap;
// use crate::compiler::compiled::Compiled;
use crate::compiler::blocks::block::Block;


pub mod compiler;

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

#[wasm_bindgen]
pub fn compile(poses:JsValue) -> JsValue {
    let blocks: Vec<Block> = poses.into_serde().unwrap_or(Vec::new());
    return JsValue::from_serde(&blocks).unwrap()
}

