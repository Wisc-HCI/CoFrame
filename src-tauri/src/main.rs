#![cfg_attr(
  all(not(debug_assertions), target_os = "windows"),
  windows_subsystem = "windows"
)]

use std::{
  collections::HashMap,
  sync::Mutex,
  // thread::{sleep, spawn},
  // time::{Instant,Duration},
};

struct Storage(Mutex<HashMap<String,String>>);

#[tauri::command]
fn set_item(state: tauri::State<Storage>, key: String, value: String) {
  println!("Setting item for {:?}", key);
  state.0.lock().unwrap().insert(key,value);
  return
}

#[tauri::command]
fn get_item(state: tauri::State<Storage>, key: String) -> Option<String> {
  // return state.0.lock().unwrap_or(None)//.get(&key).map(|result| result.clone());
  println!("Getting item for {:?}", key);
  return state.0.lock().unwrap().get(&key).map(|result| result.clone());
}

#[tauri::command]
fn remove_item(state: tauri::State<Storage>, key: String) {
  println!("Deleting item for {:?}", key);
  state.0.lock().unwrap().remove(&key);
  return
}

fn main() {
  tauri::Builder::default()
    .manage(Storage(Mutex::new(HashMap::new())))
    .invoke_handler(tauri::generate_handler![
      set_item,
      get_item,
      remove_item
    ])
    .run(tauri::generate_context!())
    .expect("error while running tauri application");
}
