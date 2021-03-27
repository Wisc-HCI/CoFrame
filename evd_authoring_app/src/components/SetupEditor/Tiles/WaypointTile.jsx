import React from 'react';

import { 
    Stack,
    TextField,
    PrimaryButton
} from 'office-ui-fabric-react';

import { DeleteButton } from '../DeleteButton';
import { ElementTile } from '../ElementTile';


export const WaypointTile = (props) => {

    const { style, name, uuid, deleteCallback } = props;

    return (
        <ElementTile style={style}>
            <Stack horizontal tokens={{childrenGap: '50px'}}>
                <Stack.Item grow styles={{root: { marginLeft: '10px'}}}>
                    <TextField label="Name:" onChange={() => {}} defaultValue={name} styles={{root: { maxWidth: '400px'}}}/>
                </Stack.Item>
                <Stack.Item align="center">
                    <PrimaryButton text="Edit Pose" onClick={() => {}} />
                </Stack.Item>
                <Stack.Item align="center" styles={{root: { marginRight: '10px'}}}>
                    <DeleteButton type="Waypoint" callback={() => { deleteCallback(uuid); }} />
                </Stack.Item>
            </Stack>
        </ElementTile>
    );
};