import React from 'react';

import {
    Stack,
    TextField,
    PrimaryButton,
    Dropdown
} from 'office-ui-fabric-react';

import { DeleteButton } from '../DeleteButton';
import { ElementTile } from '../ElementTile';


export const RegionTile = (props) => {

    const { style, name, uuid, deleteCallback, canDelete, canEdit } = props;

    const options = [
        { key: 'orientation', text: 'Orientation-Uncertainty-Only'},
        { key: 'box', text: 'Box-Pose-Uncertainty'},
        { key: 'sphere', text: 'Sphere-Pose-Uncertainty'}
    ];

    const selectedOption = "orientation";

    return (
        <ElementTile style={style}>
            <Stack horizontal tokens={{childrenGap: '50px'}}>
                <Stack.Item grow styles={{root: { marginLeft: '10px'}}}>
                    <TextField label="Name:" onChange={() => {}} defaultValue={name} styles={{root: { maxWidth: '400px'}}}/>
                </Stack.Item>
                <Stack.Item>
                    <Dropdown 
                        label="Type"
                        selectedKey={selectedOption}
                        onChange={() => {}}
                        options={options}
                    />
                </Stack.Item>
                <Stack.Item align="center">
                    <PrimaryButton text="Edit Pose" onClick={() => {}}  disabled={!canEdit} />
                </Stack.Item>
                <Stack.Item align="center" styles={{root: { marginRight: '10px'}}}>
                    <DeleteButton type="Region" callback={() => { deleteCallback(uuid) }} disabled={!canDelete} />
                </Stack.Item>
            </Stack>
        </ElementTile>
    );
};