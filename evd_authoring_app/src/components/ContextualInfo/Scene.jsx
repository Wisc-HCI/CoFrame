import React from 'react';
// import { Tooltip } from 'antd';
import { Blurb } from './Blurb';

const ROBOT_DATA = {
    name: "Robot",
    description: "The robot that can be programmed. This is a UR3e Series that can carry 3kg of payload."
}
const GRIPPER_DATA = {
    name: "Gripper",
    description: "The gripper of the robot. This is a 2-finger Robotiq 85 gripper."
}

const INFO = {
    table: {
        name: "Table",
        description: "The table is the main workspace of the program."
    },
    box: {
        name: "Box",
        description: "The box is where finished products are placed."
    },
    pedestal: {
        name: "Box",
        description: "The box is where finished products are placed."
    },
    printer: {
        name: '3D Printer',
        description: "This is a 3D printer capable of producing certain parts for assembly."
    },
    base: ROBOT_DATA,
    shoulderLink: ROBOT_DATA,
    upperArmLink: ROBOT_DATA,
    forearmLink: ROBOT_DATA,
    wrist1Link: ROBOT_DATA,
    wrist2Link: ROBOT_DATA,
    wrist3Link: ROBOT_DATA,
    gripperBaseLink: GRIPPER_DATA,
    gripperLeftKnuckle: GRIPPER_DATA,
    gripperRightKnuckle: GRIPPER_DATA,
    gripperLeftFinger: GRIPPER_DATA,
    gripperRightFinger: GRIPPER_DATA,
    gripperLeftInnerKnuckle: GRIPPER_DATA,
    gripperRightInnerKnuckle: GRIPPER_DATA,
    gripperLeftFingerTip: GRIPPER_DATA,
    gripperRightFingerTip: GRIPPER_DATA
}

export function getSceneInfo({editorPane,setupTab,frame,primaryColor,focusData,secondaryFocusData,currentIssue}) {
    console.log(focusData)
    let tabs = [];
    if (currentIssue) {
        tabs.push(
            {
                title: currentIssue.title, 
                contents:<div>
                    <Blurb highlight={primaryColor}>
                        <h3>{currentIssue.description}</h3>
                        {currentIssue.requiresChanges && (
                            <span>
                                <br></br>
                                <br></br>
                                <i>Since this issue is required, you will need to address this issue in order to simulate.</i>
                            </span>
                        )}
                    </Blurb>
                </div>
            }
        )
    }
    tabs.push(
        {
            title:<span>{INFO[focusData.uuid].name}</span>,
            contents:<div>
                {focusData && (
                    <Blurb highlight="rgb(50,50,50)">
                        <h3>About the {INFO[focusData.uuid].name}</h3>
                        {INFO[focusData.uuid].description}
                    </Blurb>
                )}
            </div>
        }
    );
    return tabs;
}