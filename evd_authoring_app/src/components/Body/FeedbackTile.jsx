import React from 'react';
import { Card, Alert } from 'antd';
import useGuiStore from '../../stores/GuiStore';

export function FeedbackTile(_) {

    const { editorPane, setupTab, focusItem, frame, primaryColor } = useGuiStore(state=>({
        editorPane:state.editorPane,
        setupTab:state.setupTab,
        focusItem:state.focusItem,
        frame:state.frame,
        primaryColor:state.primaryColor
    }))

    let title = '';
    let description = <p></p>

    if (editorPane === 'program') {
        title = 'Info: The Program Editor';
        description = <p>You can adjust the program in the Program Editor by dragging elements from the drawer into the canvas. Elements can be modified by clicking on them from within the canvas.</p>
    } else if (editorPane === 'setup' && focusItem.type === null) {
        if (setupTab === 'locations') {
            title = 'Info: Locations';
            description = <p>Locations are meaningful positions in the scene. For example, they can be used to define goals for placing or picking up <i>Things</i>, or specifying starting or ending positions for the robot.</p>
        } else if (setupTab === 'machines') {
            title = 'Info: Machines';
            description = <p>Machines are items in the workspace that perform tasks, such as modifying or combining <i>Things</i> after placement in defined configurations. They also encode the duration for each process.</p>
        } else if (setupTab === 'waypoints') {
            title = 'Info: Waypoints';
            if (frame ===  'safety') {
                description = (
                    <p>
                        Waypoints are used to construct <i style={{color:primaryColor}}>Trajectories</i> that the robot follows to complete activities.<br/>
                        <Alert showIcon message={'Pay special attention to placing waypoints around the occupancy zone of the human, since this is more likely to result in undesirable conflicts between the human and the robot.'}></Alert>
                    </p>
                )
                
            } else if (frame === 'performance') {
                description = (
                    <p>
                        Waypoints are used to construct <i style={{color:primaryColor}}>Trajectories</i> that the robot follows to complete activities.<br/>
                        <Alert showIcon message={'Pay special attention to the sequences of waypoints and where they are relative to one another within a trajectory. Longer trajectories take longer to execute and can contribute to greater space usage.'}></Alert>
                    </p>
                )
            } else {
                description = <p>Waypoints are used to construct <i style={{color:primaryColor}}>Trajectories</i> that the robot follows to complete activities.</p>
            }
            
        } else if (setupTab === 'thingTypes') {

        }
    } else if (focusItem.type !== null) {
        title = 'Info: Detail';
        description = <p>Specific info for this thing</p>
    }


    return (
        <Card title={title} style={{flex:1}}>
            {description}
        </Card>
    )
}
