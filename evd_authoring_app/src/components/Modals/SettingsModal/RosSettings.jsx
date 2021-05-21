import React from 'react';

import { TextField } from '@fluentui/react/lib/TextField';
import { Label } from '@fluentui/react/lib/Label';
import { Toggle } from '@fluentui/react/lib/Toggle';
import { Spinner, SpinnerSize } from '@fluentui/react/lib/Spinner';

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

        // You can remove the consumers and useContexts
    
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


