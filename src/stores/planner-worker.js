import * as Comlink from 'comlink';

const loadModule = async () => {
    const module = await import('@people_and_robots/lively_tk');
    return module;
  }

export const performPoseProcess = (data) => {
    const { urdf, pose, scene } = data;
    // Process the data without stalling the UI
    loadSolver().then(module=>{
        const solver = new module.Solver(urdf,[
            {type:'PositionMatch',name:"EE Position",link:"panda_hand",weight:50},
            {type:'OrientationMatch',name:"EE Rotation",link:"panda_hand",weight:25},
            {type:'SmoothnessMacro',name:"General Smoothness",weight:10},
            {type:'CollisionAvoidance',name:"Collision Avoidance",weight:10}
          ],[]);
          return solver
    })
    fibonacci(45);
  
    return data;
}

const performTrajectoryProcess = (data) => {
    const { urdf, action, trajectory, waypoints, locations, scene } = data;
    // Process the data without stalling the UI
    fibonacci(45);
  
    return data;
}

Comlink.expose({performPoseProcess, performTrajectoryProcess})