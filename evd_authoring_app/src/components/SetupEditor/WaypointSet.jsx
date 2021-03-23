import React from 'react';

import { Stack } from 'office-ui-fabric-react';

import { Set } from './Set';
import { AddButton } from './AddButton';
import { WaypointTile } from './WaypointTile';


export const WaypointSet = (props) => {

    const padding = 10;

    let waypoints = [];

    for (let i=0; i<20; i++) {
        waypoints.push({
            uuid: i,
            name: `Waypoint-${i}`
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
                <AddButton type="Waypoint" callback={() => {}} />
            </Stack.Item>

            {waypoints.map(w => (
                <Stack.Item 
                    align="stretch"
                    styles={{
                        root: {
                            paddingBottom: `${padding}px`
                        }
                    }}
                >
                    <WaypointTile 
                        uuid={w.uuid} 
                        name={w.name} 
                        deleteCallback={(uuid) => {}} 
                    />
                </Stack.Item>
            ))}

        </Set>
        
    );
};