import React from 'react';

import { Stack } from 'office-ui-fabric-react';

import { Set } from './Set';
import { AddButton } from './AddButton';
import { LocationTile } from './LocationTile';


export const LocationSet = (props) => {

    const padding = 10;

    let locations = [];

    for (let i=0; i<20; i++) {
        locations.push({
            uuid: i,
            name: `Location-${i}`,
            canDelete: ! (i === 3)
        });
    }

    return (

        <Set>

            <Stack.Item 
                align="stretch" 
                styles={{
                    root: {
                        paddingBottom: `${padding}px`
                    }
                }}
            >
                <AddButton type="Location" callback={() => {}} />
            </Stack.Item>

            {locations.map(l => (
                <Stack.Item 
                    align="stretch"
                    styles={{
                        root: {
                            paddingBottom: `${padding}px`
                        }
                    }}
                >
                    <LocationTile 
                        uuid={l.uuid} 
                        name={l.name} 
                        deleteCallback={(uuid) => {}} 
                        canDelete={l.canDelete}
                    />
                </Stack.Item>
            ))}

        </Set>
    );
};