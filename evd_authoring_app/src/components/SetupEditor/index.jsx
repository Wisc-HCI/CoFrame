import React from 'react';

import { 
    Stack, 
    Separator, 
    Text 
} from 'office-ui-fabric-react';


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
            { name: 'Locations', key: 'locations' },
            { name: 'Waypoints', key: 'waypoints' },
            { name: 'Regions', key: 'regions' },
            { name: 'Things', key: 'things' },
            { name: 'Machines', key: 'machines' },
            { name: 'environment', key: 'environment' }
        ];


        let content = null;
        switch (activeField) {
            case "locations":
                content = "locations";
                break;
            case "waypoints":
                content = "waypoints";
                break;
            case "regions":
                content = "regions";
                break;
            case "things":
                content = "things";
                break;
            case "machines":
                content = "machines";
                break;
            case "environment":
                content = "environment";
                break;
            default:
                break;
        }

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
                                    background: (entry.key === activeField) ? '#1f1f1f' : 'inherit'
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