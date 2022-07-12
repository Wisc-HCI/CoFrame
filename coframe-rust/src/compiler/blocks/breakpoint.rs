use std::collections::HashMap;

pub struct BreakpointBlock {
    pub name: String,
    pub description: String,
    pub compiled: HashMap<String,String>
}