import React from 'react';

import { 
    Stack, 
    Separator, 
    Text 
} from 'office-ui-fabric-react';

import { Set } from './Set';

import { LocationTile } from './Tiles/LocationTile';
import { WaypointTile } from './Tiles/WaypointTile';
import { RegionTile } from './Tiles/RegionTile';
import { MachineTile } from './Tiles/MachineTile';
import { ThingTile } from './Tiles/ThingTile';
import { ThingTypeTile } from './Tiles/ThingTypeTile';


export class SetupEditor extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            activeField: 'locations'
        };

        this.onLinkClick = this.onLinkClick.bind(this);
    }

    onLinkClick(item) {
        console.log(item);

        if (item) {
            this.setState({activeField: item});
        }
    }

    render() {

        const { 
            width, 
            height 
        } = this.props;
        
        const { activeField } = this.state;

        //===========================================
        // Replace with evd context

        let waypoints = [];
        for (let i=0; i<20; i++) {
            waypoints.push({
                uuid: i,
                name: `Waypoint-${i}`
            });
        }

        let regions = [];
        for (let i=0; i<30; i++) {
            regions.push({
                uuid: i,
                name: `Region-${i}`,
                canDelete: i % 3,
                canEdit: i % 3
            });
        }

        let locations = [];
        for (let i=0; i<20; i++) {
            locations.push({
                uuid: i,
                name: `Location-${i}`,
                canDelete: ! (i === 3),
                canEdit: ! (i % 4 === 0)
            });
        }
        
        let machines = [];
        for (let i=0; i<30; i++) {
            machines.push({
                uuid: i,
                name: `Machine-${i}`,
                canDelete: i % 3,
                canEdit: i % 3,
                mesh: 'default.stl',
                inputs: [],
                outputs: []
            });
        }

        let thingTypes = [];
        for (let i=0; i<5; i++) {
            thingTypes.push({
                uuid: i,
                name: `Thing-Type-${i}`,
                canDelete: false,
                mesh: 'default.stl'
            });
        }

        let things = [];
        for (let i=0; i<10; i++) {
            things.push({
                uuid: i,
                name: `Thing-${i}`,
                canDelete: false,
                canEdit: true
            });
        }

        //============================================


        const groups = [
            { name: 'Locations', key: 'locations', content: (
                <Set 
                    sets={[
                        { name: 'Locations', type: 'Location', content: locations.map(l => (
                            <div key={l.uuid} style={{ paddingBottom: '10px' }}>
                                <LocationTile 
                                    uuid={l.uuid} 
                                    name={l.name} 
                                    deleteCallback={(uuid) => {}} 
                                    canDelete={l.canDelete}
                                    canEdit={l.canEdit}
                                />
                            </div>
                        ))}
                    ]}
                />
            )},

            { name: 'Waypoints', key: 'waypoints', content: (
                <Set 
                    sets={[
                        { name: 'Waypoints', type: 'Waypoint', content: waypoints.map(w => (
                            <div key={w.uuid} style={{ paddingBottom: '10px' }}>
                                <WaypointTile 
                                    uuid={w.uuid} 
                                    name={w.name} 
                                    deleteCallback={(uuid) => {}} 
                                />
                            </div>
                        ))}
                    ]}
                />
            )},

            { name: 'Regions', key: 'regions', content: (
                <Set  
                    sets={[
                        { name: 'Regions', type: 'Region', content: regions.map(r => (
                            <div key={r.uuid} style={{ paddingBottom: '10px' }}>
                                <RegionTile 
                                    uuid={r.uuid} 
                                    name={r.name} 
                                    deleteCallback={(uuid) => {}} 
                                    canDelete={r.canDelete}
                                    canEdit={r.canEdit}
                                />
                            </div>
                        ))}
                    ]}
                />
            )},

            { name: 'Things', key: 'things', content: (
                <Set 
                    sets={[
                        { name: 'Thing Types', type: "Type", content: thingTypes.map(t => (
                            <div key={t.uuid} style={{ paddingBottom: '10px' }}>
                                <ThingTypeTile 
                                    uuid={t.uuid}
                                    name={t.name}
                                    deleteCallback={(uuid) => {}}
                                    canDelete={t.canDelete}
                                    mesh={t.mesh}
                                />
                            </div>
                        ))},
                        { name: 'Thing Instances', type: "Instance", content: things.map(t => (
                            <div key={t.uuid} style={{ paddingBottom: '10px' }}>
                                <ThingTile 
                                    uuid={t.uuid}
                                    name={t.name}
                                    deleteCallback={(uuid) => {}}
                                    canDelete={t.canDelete}
                                    canEdit={t.canEdit}
                                    thingTypes={thingTypes}
                                />
                            </div>
                        ))}
                    ]}
                />
            )},

            { name: 'Machines', key: 'machines', content: (
                <Set 
                    sets={[
                        { name: 'Machines', type: 'Machine', content: machines.map(m => (
                            <div key={m.uuid} style={{ paddingBottom: '10px' }}>
                                <MachineTile 
                                    uuid={m.uuid} 
                                    name={m.name} 
                                    deleteCallback={(uuid) => {}} 
                                    canDelete={m.canDelete}
                                    canEdit={m.canEdit}
                                    mesh={m.mesh}
                                    inputs={m.inputs}
                                    outputs={m.outputs}
                                    thingTypes={thingTypes}
                                    regions={regions}
                                />
                            </div>
                        ))}
                    ]}
                />
            )},

            { name: 'Environment', key: 'environment', content: (
                <Set 
                    sets={[
                        { name: 'Collision Objects', type: 'Object', content: []},
                        { name: 'Pinch Points', type: 'Point', content: []},
                        { name: 'Occupancy Zones', type: 'Zone', content: []}
                    ]}
                />
            )}
        ];

        return (
            <div
                style={{ 
                    height: `${height}px`, 
                    width: `${width}px`,
                    padding: '5px'
                }}
            >
                <Stack horizontal 
                    style={{
                        height: '100%', 
                        width: '100%'
                    }}
                >

                    <Stack.Item>
                        {groups.map(entry => (
                            <div 
                                key={entry.key} 
                                onClick={e => this.onLinkClick(entry.key)} 
                                style={{
                                    cursor: 'pointer', 
                                    background: (entry.key === activeField) ? '#2f2f2f' : 'inherit'
                                }}
                            >
                                <Text variant='xLarge' 
                                    style={{
                                        margin: '10px'
                                    }}
                                >
                                    {entry.name}
                                </Text>
                            </div>
                        ))}
                    </Stack.Item>
                    
                    <Separator vertical />

                    {groups.find(x => x.key === activeField).content}
                    
                </Stack>
                
            </div>
        );
    }
}