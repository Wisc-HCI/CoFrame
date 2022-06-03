use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize,Clone,Debug)]
pub enum Status {
    PENDING,
    VALID,
    FAILED,
    WARN
}