import React from 'react';

import { 
    Stack, 
    Separator, 
    Text 
} from 'office-ui-fabric-react';

import { LocationSet } from './LocationSet';
import { WaypointSet } from './WaypointSet';
import { RegionSet } from './RegionSet';
import { ThingSet } from './ThingSet';
import { MachineSet } from './MachineSet';
import { EnvironmentSet } from './EnvironmentSet';



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


        const groups = [
            { name: 'Locations', key: 'locations', content: (<LocationSet />) },
            { name: 'Waypoints', key: 'waypoints', content: (<WaypointSet />) },
            { name: 'Regions', key: 'regions', content: (<RegionSet />) },
            { name: 'Things', key: 'things', content: (<ThingSet />) },
            { name: 'Machines', key: 'machines', content: (<MachineSet />) },
            { name: 'Environment', key: 'environment', content: (<EnvironmentSet />) }
        ];


        let content = groups.find(x => x.key === activeField).content;

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

                    {content}
                    
                </Stack>
                
            </div>
        );
    }
}