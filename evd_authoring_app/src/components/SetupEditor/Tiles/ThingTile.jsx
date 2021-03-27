import React from 'react';

import { 
    Stack,
    TextField,
    PrimaryButton,
    Dropdown
} from 'office-ui-fabric-react';

import { DeleteButton } from '../DeleteButton';
import { ElementTile } from '../ElementTile';


export const ThingTile = (props) => {

    const { style, name, uuid, deleteCallback, canDelete, canEdit, thingTypes } = props;

    const options = thingTypes.map(t => {
        return {
            key: t.uuid,
            text: t.name
        };
    });

    const selectedOption = options[0];

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
                    <DeleteButton type="Waypoint" callback={() => { deleteCallback(uuid) }} disabled={!canDelete} />
                </Stack.Item>
            </Stack>
        </ElementTile>
    );
};