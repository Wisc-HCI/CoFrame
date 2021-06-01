import React from 'react';

import { Stack } from '@fluentui/react/lib/Stack';
import { TextField } from '@fluentui/react/lib/TextField';

import { DeleteButton } from '../DeleteButton';
import { ElementTile } from '../ElementTile';

export const ThingTypeTile = (props) => {

    const { style, name, uuid, deleteCallback, mesh, canDelete } = props;

    return (
        <ElementTile style={style}>
            <Stack horizontal tokens={{childrenGap: '50px'}}>
                <Stack.Item grow styles={{root: { marginLeft: '10px'}}}>
                    <TextField label="Name:" onChange={() => {}} defaultValue={name} styles={{root: { maxWidth: '400px'}}} />
                </Stack.Item>
                <Stack.Item grow >
                    <TextField label="Mesh:" onChange={() => {}} defaultValue={mesh} styles={{root: { maxWidth: '400px'}}} />
                </Stack.Item>
                <Stack.Item align="center" styles={{root: { marginRight: '10px'}}}>
                    <DeleteButton type="Waypoint" callback={() => { deleteCallback(uuid) }} disabled={!canDelete} />
                </Stack.Item>
            </Stack>
        </ElementTile>
    );
};