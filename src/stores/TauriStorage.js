// import { invoke } from "@tauri-apps/api";

// const immer = (config) => (set, get, api) =>
//   config(
//     (partial, replace) => {
//       const nextState =
//         typeof partial === "function" ? produce(partial) : partial;
//       return set(nextState, replace);
//     },
//     get,
//     api
// );

// export const TauriStorage = {
//   getItem: (key) => invoke('get_item',{key}),
//   setItem: (key,value) => invoke('set_item',{key,value}),
//   removeItem: (key) => invoke('remove_item',{key})
// }