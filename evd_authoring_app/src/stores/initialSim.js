export const INITIAL_SIM =
    {
        tfs:{
            'simulated_base_link':{
                frame:'world',
                translation: { x: 0, y: 0, z: 0 },
                rotation: { w: 0, x: 0, y: 0, z: 1 },
            },
            'simulated_shoulder_link':{
                frame:'simulated_base_link',
                translation: { x: 0, y: 0, z: 0.15185 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
            },
            'simulated_upper_arm_link':{
                frame:'simulated_shoulder_link',
                translation: { x: 0, y: 0, z: 0 },
                rotation: { w: 0.7071067811140325, x: 0.7071067812590626, y: 0, z: 0 }
            },
            'simulated_forearm_link':{
                frame:'simulated_upper_arm_link',
                translation: { x: -0.24355, y: 0, z: 0 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
            },
            'simulated_wrist_1_link':{
                frame:'simulated_forearm_link',
                translation: { x: -0.2132, y: 0, z: 0.13105},
                rotation: { w: 1, x: 0, y: 0, z: 0 },
            },
            'simulated_wrist_2_link':{
                frame:'simulated_wrist_1_link',
                translation: { x: 0, y: -0.08535, z: 0 },
                rotation: { w: 0.7071067811140325, x: 0.7071067812590626, y: 0, z: 0 },
            },
            'simulated_wrist_3_link':{
                frame:'simulated_wrist_2_link',
                translation: { x: 0, y: 0.0921, z: 0 },
                rotation: { w: 0.7071067811140325, x: -0.7071067812590626, y: 0, z: 0 },
            },
            'simulated_flange':{
                frame:'simulated_wrist_3_link',
                translation: { x: 0, y: 0, z: 0 },
                rotation: { w: 0.5, x: -0.5, y: -0.5, z: -0.5 },
              },
            'simulated_tool0':{
                frame:'simulated_flange',
                translation: { x: 0, y: 0, z: 0 },
                rotation: { w: 0.5, x: 0.5, y: 0.5, z: 0.5 },
            },
            'simulated_robotiq_85_base_link':{
                frame:'simulated_tool0',
                translation: {x: 0, y: 0, z: 0},
                rotation: { w: 0.5, x: 0.5, y: -0.5, z: 0.5},
            },
            'simulated_robotiq_85_left_knuckle_link':{
                frame:'simulated_robotiq_85_base_link',
                translation: { x: 0.05490451627, y: 0.03060114443, z: 0 },
                rotation: { w: 0, x: 1, y: 0, z: 0 },
            },
            'simulated_robotiq_85_right_knuckle_link':{
                frame:'simulated_robotiq_85_base_link',
                translation: { x: 0.05490451627, y: -0.03060114443, z: 0 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
            },
            'simulated_robotiq_85_left_finger_link':{
                frame:'simulated_robotiq_85_left_knuckle_link',
                translation: { x: -0.00408552455, y: -0.03148604435, z: 0 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
            },
            'simulated_robotiq_85_right_finger_link':{
                frame:'simulated_robotiq_85_right_knuckle_link',
                translation: { x: -0.00408552455, y: -0.03148604435, z: 0 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
            },
            'simulated_robotiq_85_left_inner_knuckle_link':{
                frame:'simulated_robotiq_85_base_link',
                translation: { x: 0.06142, y: 0.0127, z: 0 },
                rotation: { w: 0, x: 1, y: 0, z: 0 },
            },
            'simulated_robotiq_85_right_inner_knuckle_link':{
                frame:'simulated_robotiq_85_base_link',
                translation: { x: 0.06142, y: -0.0127, z: 0 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
            },
            'simulated_robotiq_85_left_finger_tip_link':{
                frame:'simulated_robotiq_85_left_inner_knuckle_link',
                translation: { x: 0.04303959807, y: -0.03759940821, z: 0 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
            },
            'simulated_robotiq_85_right_finger_tip_link':{
                frame:'simulated_robotiq_85_right_inner_knuckle_link',
                translation: { x: 0.04303959807, y: -0.03759940821, z: 0 },
                rotation: { w: 1, x: 0, y: 0, z: 0},
            }
        },
        staticScene: {
            table: {
                visual: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/Table/Table.stl",
                collision: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/Table/Table.stl",
                name: "Table",
                frame: "world",
                position: { x: 0, y: 0.36, z: -0.37 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
                color: {r: 40, g: 40, b: 40, a: 1},
                showCollision: false,
                highlighted: false,
                scale: {x:1,y:1,z:1}
            },
            pedestal: {
                visual: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/ur3e-Pedestal/Pedestal.stl",
                collision: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/Pedestal.stl",
                name: "Pedestal",
                frame: "world",
                position: { x: 0, y: 0, z: -0.38 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
                color: {r: 50, g: 50, b: 50, a: 1},
                showCollison: false,
                highlighted: false,
                scale: {x:1,y:1,z:1}
            },
            box: {
                visual: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/Box/Box.stl",
                collision: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/Box.stl",
                name: "Box",
                frame: "world",
                position: { x: 0.35, y: 0.35, z: 0.07 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
                showCollison: false,
                highlighted: false,
                scale: {x:1,y:1,z:1}
            },
            printer: {
                visual: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/MK2-Printer/MK2-Printer.stl",
                collision: "package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/MK2-Printer.stl",
                name: "3D Printer",
                frame: "world",
                position: { x: -0.28, y: 0.32, z: 0.3 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
                showCollison: false,
                highlighted: false,
                scale: {x:1,y:1,z:1}
            },
            robotBase: {
                visual: "package://ur_description/meshes/ur3/visual/base.dae",
                collision: "package://ur_description/meshes/ur3/collision/base.stl",
                name: 'Robot Base',
                frame: "simulated_base_link",
                position: { x: 0, y: 0, z: 0 },
                rotation: { w: 0, x: 0, y: 0, z: 1 },
                scale: {x:1,y:1,z:1},
                showCollison: false,
                highlighted: false
            },
            robotShoulderLink: {
                visual: "package://ur_description/meshes/ur3/visual/shoulder.dae",
                collision: "package://ur_description/meshes/ur3/collision/shoulder.stl",
                name: 'Shoulder Link',
                frame: "simulated_shoulder_link",
                position: { x: 0, y: 0, z: 0 },
                rotation: { w: 0, x: 0, y: 0, z: 1 },
                scale: {x:1,y:1,z:1},
                showCollison: false,
                highlighted: false,
            },
            robotUpperArmLink: {
                visual: "package://ur_description/meshes/ur3/visual/upperarm.dae",
                collision: "package://ur_description/meshes/ur3/collision/upperarm.stl",
                name: "Upper Arm Link",
                frame: "simulated_upper_arm_link",
                position: { x: 0, y: 0, z: 0.12 },
                rotation: { w: 0.5, x: 0.5, y: -0.5, z: -0.5 },
                scale: {x:1,y:1,z:1},
                showCollison: false,
                highlighted: false,
            },
            robotForearmLink: {
                visual: "package://ur_description/meshes/ur3/visual/forearm.dae",
                collision: "package://ur_description/meshes/ur3/collision/forearm.stl",
                name: "Forearm Link",
                frame: "simulated_forearm_link",
                position: { x: 0, y: 0, z: 0.027 },
                rotation: { w: 0.5, x: 0.5, y: -0.5, z: -0.5 },
                scale: {x:1,y:1,z:1},
                showCollison: false,
                highlighted: false,
            },
            robotWrist1Link: {
                visual: "package://ur_description/meshes/ur3/visual/wrist1.dae",
                collision: "package://ur_description/meshes/ur3/collision/wrist1.stl",
                name: "Wrist 1 Link",
                frame: "simulated_wrist_1_link",
                position: { x: 0, y: 0, z: -0.104 },
                rotation: { w: 0.7071068, x: 0.7071068, y: 0, z: 0 },
                scale: {x:1,y:1,z:1},
                showCollison: false,
                highlighted: false,
            },
            robotWrist2Link: {
                visual: "package://ur_description/meshes/ur3/visual/wrist2.dae",
                collision: "package://ur_description/meshes/ur3/collision/wrist2.stl",
                name: "Wrist 2 Link",
                frame: "simulated_wrist_2_link",
                position: { x: 0, y: 0, z: -0.08535 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
                scale: {x:1,y:1,z:1},
                showCollison: false,
                highlighted: false,
            },
            robotWrist3Link: {
                visual: "package://ur_description/meshes/ur3/visual/wrist3.dae",
                collision: "package://ur_description/meshes/ur3/collision/wrist3.stl",
                name: "Wrist 3 Link",
                frame: "simulated_wrist_3_link",
                position: { x: 0, y: 0, z: -0.0921 },
                rotation: { w: 0.7071068, x: 0.7071068, y: 0, z: 0 },
                scale: {x:1,y:1,z:1},
                showCollison: false,
                highlighted: false,
            },
            gripperBaseLink:{
                visual: "package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae",
                collision: "package://robotiq_85_description/meshes/collision/robotiq_85_base_link.stl",
                name: "Gripper Base",
                frame: "simulated_robotiq_85_base_link",
                position: { x: 0, y: 0, z: 0},
                rotation: { w: 1, x: 0, y: 0, z: 0},
                scale: {x:1,y:1,z:1},
                showName: false,
                highlighted: false
              },
              gripperLeftKnuckle:{
                visual: "package://robotiq_85_description/meshes/visual/robotiq_85_knuckle_link.dae",
                collision: "package://robotiq_85_description/meshes/collision/robotiq_85_knuckle_link.stl",
                name: "Gripper Left Knuckle",
                frame: "simulated_robotiq_85_left_knuckle_link",
                position: { x: 0, y: 0, z: 0},
                rotation: { w: 1, x: 0, y: 0, z: 0},
                scale: {x:1,y:1,z:1},
                showName: false,
                highlighted: false
              },
              gripperRightKnuckle:{
                visual: "package://robotiq_85_description/meshes/visual/robotiq_85_knuckle_link.dae",
                collision: "package://robotiq_85_description/meshes/collision/robotiq_85_knuckle_link.stl",
                name: "Gripper Right Knuckle",
                frame: "simulated_robotiq_85_right_knuckle_link",
                position: { x: 0, y: 0, z: 0},
                rotation: { w: 1, x: 0, y: 0, z: 0},
                scale: {x:1,y:1,z:1},
                showName: false,
                highlighted: false
              },
              gripperLeftFinger:{
                visual: "package://robotiq_85_description/meshes/visual/robotiq_85_finger_link.dae",
                collision: "package://robotiq_85_description/meshes/collision/robotiq_85_finger_link.stl",
                name: "Gripper Left Finger",
                frame: "simulated_robotiq_85_left_finger_link",
                position: { x: 0, y: 0, z: 0},
                rotation: { w: 1, x: 0, y: 0, z: 0},
                scale: {x:1,y:1,z:1},
                showName: false,
                highlighted: false
              },
              gripperRightFinger:{
                visual: "package://robotiq_85_description/meshes/visual/robotiq_85_finger_link.dae",
                collision: "package://robotiq_85_description/meshes/collision/robotiq_85_finger_link.stl",
                name: "Gripper Right Finger",
                frame: "simulated_robotiq_85_right_finger_link",
                position: { x: 0, y: 0, z: 0},
                rotation: { w: 1, x: 0, y: 0, z: 0},
                scale: {x:1,y:1,z:1},
                showName: false,
                highlighted: false
              },
              gripperLeftInnerKnuckle:{
                visual: "package://robotiq_85_description/meshes/visual/robotiq_85_inner_knuckle_link.dae",
                collision: "package://robotiq_85_description/meshes/collision/robotiq_85_inner_knuckle_link.stl",
                name: "Gripper Left Inner Knuckle",
                frame: "simulated_robotiq_85_left_inner_knuckle_link",
                position: { x: 0, y: 0, z: 0},
                rotation: { w: 1, x: 0, y: 0, z: 0},
                scale: {x:1,y:1,z:1},
                showName: false,
                highlighted: false
              },
              gripperRightInnerKnuckle:{
                visual: "package://robotiq_85_description/meshes/visual/robotiq_85_inner_knuckle_link.dae",
                collision: "package://robotiq_85_description/meshes/collision/robotiq_85_inner_knuckle_link.stl",
                name: "Gripper Right Inner Knuckle",
                frame: "simulated_robotiq_85_right_inner_knuckle_link",
                position: { x: 0, y: 0, z: 0},
                rotation: { w: 1, x: 0, y: 0, z: 0},
                scale: {x:1,y:1,z:1},
                showName: false,
                highlighted: false
              },
              gripperLeftFingerTip:{
                visual: "package://robotiq_85_description/meshes/visual/robotiq_85_finger_tip_link.dae",
                collision: "package://robotiq_85_description/meshes/collision/robotiq_85_finger_tip_link.stl",
                name: "Gripper Left Finger Tip",
                frame: "simulated_robotiq_85_left_finger_tip_link",
                position: { x: 0, y: 0, z: 0},
                rotation: { w: 1, x: 0, y: 0, z: 0},
                scale: {x:1,y:1,z:1},
                showName: false,
                highlighted: false
              },
              gripperRightFingerTip:{
                visual: "package://robotiq_85_description/meshes/visual/robotiq_85_finger_tip_link.dae",
                collision: "package://robotiq_85_description/meshes/collision/robotiq_85_finger_tip_link.dae",
                name: "Gripper Right Finger Tip",
                frame: "simulated_robotiq_85_right_finger_tip_link",
                position: { x: 0, y: 0, z: 0},
                rotation: { w: 1, x: 0, y: 0, z: 0},
                scale: {x:1,y:1,z:1},
                showName: false,
                highlighted: false
              }
        },
        animationObjects: {
    
        },
        controllers: {
    
        }
}