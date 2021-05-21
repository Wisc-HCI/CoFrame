import React from 'react';

import { Stack } from '@fluentui/react/lib/Stack';
import { TextField } from '@fluentui/react/lib/TextField';
import { PrimaryButton } from '@fluentui/react/lib/Button';
import { Label } from '@fluentui/react/lib/Label';
import { Separator } from '@fluentui/react/lib/Separator';
import { SpinButton } from '@fluentui/react/lib/SpinButton';

import { RecipeEntryTile } from './RecipeEntryTile';
import { DeleteButton } from '../DeleteButton';
import { ElementTile } from '../ElementTile';


export const MachineTile = (props) => {

    const { 
        style, 
        name, 
        uuid, 
        mesh, 
        inputs, 
        outputs, 
        thingTypes, 
        regions,
        deleteCallback, 
        canDelete, 
        canEdit
    } = props;

    return (
        <ElementTile style={style}>

            <div style={{paddingLeft: '10px', paddingRight: '10px'}}>

                <Stack horizontal tokens={{childrenGap: '50px'}}>
                    <Stack.Item grow>
                        <TextField label="Name:" onChange={() => {}} defaultValue={name} styles={{root: { maxWidth: '400px'}}} disabled={!canEdit}/>
                    </Stack.Item>
                    <Stack.Item align="center">
                        <PrimaryButton text="Edit Pose" onClick={() => {}}  disabled={!canEdit} />
                    </Stack.Item>
                    <Stack.Item align="center">
                        <DeleteButton type="Region" callback={() => { deleteCallback(uuid) }} disabled={!canDelete} />
                    </Stack.Item>
                </Stack>

                <TextField label="Mesh:" onChange={() => {}} defaultValue={mesh} styles={{root: { maxWidth: '400px'}}} disabled={!canEdit}/>

                <Separator />

                <Stack horizontal tokens={{childrenGap: '50px'}}>
                    <Stack.Item grow >
                        <Label>Recipe:</Label>
                    </Stack.Item>

                    <SpinButton 
                        defaultValue="0"
                        label="Process Time:"
                        min={0}
                        step={0.5}
                        disabled={!canEdit}
                    />

                </Stack>

                <Label>Inputs:</Label>
                <Stack>

                    {inputs.map(entry => (
                        <RecipeEntryTile 
                            thingTypes={thingTypes}
                            selectedThing={entry.selectedThing}
                            quantity={entry.quantity}
                            regions={regions}
                            selectedRegion={entry.selectedRegion}
                            canEdit={canEdit}
                            canDelete={canDelete}
                        />
                    ))}

                    <Stack.Item align="center">
                        <PrimaryButton text="Add Input" onClick={() => {
                            inputs.push({
                                
                            })
                        }} disabled={!canEdit}/>
                    </Stack.Item>
                </Stack>


                <Label>Outputs:</Label>
                <Stack>

                    {outputs.map(entry => (
                        <RecipeEntryTile 
                            thingTypes={thingTypes}
                            selectedThing={entry.selectedThing}
                            quantity={entry.quantity}
                            regions={regions}
                            selectedRegion={entry.selectedRegion}
                            canEdit={canEdit}
                            canDelete={canDelete}
                        />
                    ))}

                    <Stack.Item align="center">
                        <PrimaryButton text="Add Output" onClick={() => {}} disabled={!canEdit}/>
                    </Stack.Item>
                </Stack>

                <br />

            </div>

        </ElementTile>
    );
};