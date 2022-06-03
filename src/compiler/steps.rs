use std::collections::HashMap;
use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct LandmarkStep {
    pub effect: StateProperty,
    pub data: HashMap<String,String>,
    pub source: String,
    pub delay: f64
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct SceneUpdateStep {
    pub effect: StateProperty,
    pub data: HashMap<String,String>,
    pub source: String,
    pub delay: f64
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ActionStartStep {
    pub effect: StateProperty,
    pub data: HashMap<String,String>,
    pub source: String,
    pub delay: f64
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ActionEndStep {
    pub effect: StateProperty,
    pub data: HashMap<String,String>,
    pub source: String,
    pub delay: f64
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ProcessStartStep {
    pub effect: StateProperty,
    pub data: HashMap<String,String>,
    pub source: String,
    pub delay: f64
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ProcessEndStep {
    pub effect: StateProperty,
    pub data: HashMap<String,String>,
    pub source: String,
    pub delay: f64
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct SpawnItemStep {
    pub effect: StateProperty,
    pub data: HashMap<String,String>,
    pub source: String,
    pub delay: f64
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct DestroyItemStep {
    pub effect: StateProperty,
    pub data: HashMap<String,String>,
    pub source: String,
    pub delay: f64
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(tag = "type")]
pub enum Step {
    #[serde(rename = "LANDMARK")]
    Landmark(LandmarkStep),
    #[serde(rename = "SCENE_UPDATE")]
    SceneUpdate(SceneUpdateStep),
    #[serde(rename = "ACTION_START")]
    ActionStart(ActionStartStep),
    #[serde(rename = "ACTION_END")]
    ActionEnd(ActionEndStep),
    #[serde(rename = "PROCESS_START")]
    ProcessStart(ProcessStartStep),
    #[serde(rename = "PROCESS_END")]
    ProcessEnd(ProcessEndStep),
    #[serde(rename = "SPAWN_ITEM")]
    SpawnItem(SpawnItemStep),
    #[serde(rename = "DESTROY_ITEM")]
    DestroyItem(DestroyItemStep)
}

impl Step {
    pub fn get_effect(&self) -> StateProperty {
        match self {
            Self::Landmark(step) => return step.effect.clone(),
            Self::SceneUpdate(step) => return step.effect.clone(),
            Self::ActionStart(step) => return step.effect.clone(),
            Self::ActionEnd(step) => return step.effect.clone(),
            Self::ProcessStart(step) => return step.effect.clone(),
            Self::ProcessEnd(step) => return step.effect.clone(),
            Self::SpawnItem(step) => return step.effect.clone(),
            Self::DestroyItem(step) => return step.effect.clone()
        }
    }
}

#[derive(Serialize,Deserialize,Clone,Debug)]
#[serde(untagged)]
pub enum StateProperty {
    Map(HashMap<String,StateProperty>),
    Number(f64),
    String(String),
    Bool(bool)
}

// pub impl StateProperty {
//     pub fn compare(&self,other:StateProperty) -> (bool,Vec<String>) {
//         match (self,other) {
//             (StateProperty::Map(self_map),StateProperty::Map(other_map)) => {
//                 let mut is_match: bool = true;
//                 let mut matched_keys: Vec<String> = Vec::new();

//             },
//             (StateProperty::Number(self_num),StateProperty::Number(other_num)) => {

//             },
//             (StateProperty::String(self_str),StateProperty::String(other_str)) => {

//             },
//             (StateProperty::Bool(self_bool),StateProperty::Bool(other_bool)) => {

//             },
//             (_,_) => return (false,Vec::new())
//         }
//     }

//     pub fn merge(&mut self, other:StateProperty) {
//         match (self,other) {
//             (StateProperty::Map(self_map),StateProperty::Map(other_map)) => {
//                 let mut matches: bool = true;
//                 let mut 
//             },
//             (_,_) => {self = other.clone()}
//         }
//     }
// }

// pub struct Event {
//     pub condition: StateProperty,
//     pub on_trigger: Vec<Step>,
//     pub source: String
// }

// impl Event {
//     pub fn compile_effects(&self) -> StateProperty {
//         let mut effects: HashMap<String,StateProperty> = HashMap::new();
//         for step in self.on_trigger {
//             let effect = step.get_effect();
//             match effect {
//                 StateProperty::Map(props) => {
//                     for (key, value) in props {
//                         if !effects.contains_key(&key) {
//                             effects.insert(key,value);
//                         } else {
//                             let mut current_value = effects.get_mut(&key).unwrap();
//                             current_value.extend(value)
//                         }
//                     }
//                 },
//                 _ => {}
//             }
            
//         }
//         return StateProperty::Map(effects);
//     }
// }

// pub struct ActionState {
//     pub current: StateProperty
// }

// impl ActionState {
//     pub fn new() -> Self {
//         return Self { current: StateProperty::map(HashMap::new())}
//     }
// }

// pub fn insert_event_step_at_time(current_time: f64, time_idx: usize, mut states: Vec<ActionState>, event_step: Step, condition: StateProperty) -> Vec<ActionState> {
//     return states;
// }

// pub fn search_for_time(states: Vec<ActionState>,condition:StateProperty,all_effects:StateProperty) -> (f64,bool,usize) {
//     let mut time = 0.0;
//     let mut found = states.len() == 0;
//     let mut time_idx = states.len();
//     let mut saved_matches: Vec<String> = Vec::new();
//     let mut idx: usize = -1;
//     let mut quit = false;

//     while (!quit) {
//         idx += 1;
//         let mut prev_state: ActionState;
//         if idx == 0 {
//             prev_state = ActionState::new();
//         } else {
//             prev_state = states[states.len() - idx]
//         }

//         let (current_match, current_matches) = state.current.compare(condition);

//     }
    
// //     states.
// //     .slice()
// //     .reverse()
// //     .some((state, idx) => {
// //       // console.log('considering state match for:',{condition,state,effects})
// //       const prevState =
// //         idx === 0 ? { current: {} } : states[states.length - idx];
// //       // console.log('prevState',prevState)
// //       // console.log('state',state)
// //       const [currentMatch, currentMatches] = compare(state.current, condition);
// //       const [pastMatch, _] = compare(effects, prevState.current);
// //       // console.log({currentMatch, currentMatches, pastMatch, savedMatches})
// //       if (!pastMatch || !currentMatch) {
// //         // console.log('cancelling search');
// //         return true;
// //       } else if (
// //         currentMatch &&
// //         !isSubset(currentMatches, savedMatches) &&
// //         found
// //       ) {
// //         // console.log('past match superior, cancelling');
// //         return true;
// //       } else if (currentMatch) {
// //         // console.log('match found @',states.length-idx-1)
// //         // console.log('match info:',{state:state.current,condition,prevState})
// //         time = state.time;
// //         timeIdx = states.length - idx;
// //         found = true;
// //         savedMatches = currentMatches;
// //         return false;
// //       } else {
// //         // console.log('some other condition',{currentMatch, currentMatches,pastMatch,found})
// //       }
// //     });
// //   return [time, found, timeIdx];
// }


// pub fn eventsToStates(events: Vec<Event>) -> Vec<ActionState> {
//     let mut states: Vec<ActionState> = Vec::new();
//     let mut pending_set: Vec<Event> = Vec::new();
//     for (i,event) in events.iter().enumerate() {
//         let all_effects = event.compile_effects();
//         let (current_time, found, time_idx) = search_for_time(states,event.condition,all_effects);
//         if found || states.len() == 0 {
//             for (e_idx,event_step) in event.on_trigger.iter().enumerate() {
//                 states = insert_event_step_at_time(current_time,time_idx,states,event_step,event.condition);
//             }
//             for pending_event in pending_set {
//                 let all_pending_effects = pending_event.compile_effects();
//                 let (current_pending_time, pending_found, pending_time_idx) = search_for_time(states,pending_event.condition,all_pending_effects);
//                 if (pending_found) {
//                     for (e_idx,pending_event_step) in pending_event.on_trigger.iter().enumerate() {
//                         states = insert_event_step_at_time(current_pending_time,pending_time_idx,states,pending_event_step,pending_event.condition)
//                     }
//                 }
//             }
//         } else {
//             pending_set.push(*event.clone())
//         }
//     }
//     return states
// }

// pub fn statesToSteps(states: Vec<State>) -> Vec<Step> {
//     let mut steps: Vec<Step> = Vec::new();
//     return steps
// }