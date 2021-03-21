import React from 'react';

import { 
    TextField,
    Label,
    Toggle,
    Spinner, 
    SpinnerSize
} from 'office-ui-fabric-react';

import { RosConnectButton } from './RosConnectButton';

import { RosContext } from '../../../contexts';


export class RosSettings extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            connectionState: 'connect'
        };
        
        this.textChanged = this.textChanged.bind(this);
        this.connectEvent = this.connectEvent.bind(this);
    }

    textChanged() {
        this.setState({
            connectionState: 'connect'
        });
    }

    connectEvent(service) {
        this.setState({
            connectionState: 'loading'
        });

        const url = document.getElementById("ros-url").value;
        service.onLoad(url, (val) => {
            this.setState({
                connectionState: (val) ? 'refresh' : 'connect'
            });
        });
    }

    render() {

        const { connectionState } = this.state;
    
        return (
            <RosContext.Consumer>
                { rosValue => (
                    <table className="settings-layout">
                        <tr>
                            <td className="settings-layout-first-cell">
                                <Label>Server:</Label>
                            </td>
                            <td>
                                <TextField id="ros-url"
                                    onChange={this.textChanged}
                                    defaultValue={rosValue.url}
                                    disabled={connectionState === "loading"}
                                />
                            </td>
                            <td>
                                <RosConnectButton 
                                    state={connectionState} 
                                    callback={() => {this.connectEvent(rosValue.service)}} 
                                />
                            </td>
                            <td>
                                {
                                    (connectionState === "loading") ? (<Spinner size={SpinnerSize.medium} />): null
                                }
                            </td>
                        </tr>
                        <tr>
                            <td className="settings-layout-first-cell">
                                <Label>Status:</Label>
                            </td>
                            <td>
                                <Toggle 
                                    checked={rosValue.connected}
                                    disabled 
                                    onText="Connected" 
                                    offText="Disconnected" 
                                />
                            </td>
                        </tr>
                    </table>
                )}
            </RosContext.Consumer>
        );
    }
} 


