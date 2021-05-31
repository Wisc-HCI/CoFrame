import React from 'react';
import { Card } from 'antd';
import useGuiStore from '../../stores/GuiStore';

export function FeedbackTile(_) {

    const { editorPane, setupTab, focusItem } = useGuiStore(state=>({
        editorPane:state.editorPane,
        setupTab:state.setupTab,
        focusItem:state.focusItem
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

        } else if (setupTab === 'regions') {

        } else if (setupTab === 'thingTypes') {

        } else if (setupTab === 'things') {

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
