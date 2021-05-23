import React, {useState} from 'react';

import { TextField } from '@fluentui/react/lib/TextField';
import { Label } from '@fluentui/react/lib/Label';
import { Toggle } from '@fluentui/react/lib/Toggle';
import { Spinner, SpinnerSize } from '@fluentui/react/lib/Spinner';

import { RosConnectButton } from './RosConnectButton';

import useRosStore from '../../../stores/RosStore';


export function RosSettings(_) {

    const {connectionState, setConnectionState} = useState('connect');
    const {url, setUrl, connected, connect} = useRosStore(state=>({
        url:state.url,
        setUrl:state.setUrl,
        connected:state.connected,
        connect:state.connect
    }))
    

    const connectEvent = () => {
        setConnectionState('loading');
        connect()
    }

    return (
        <table className="settings-layout">
            <tr>
                <td className="settings-layout-first-cell">
                    <Label>Server:</Label>
                </td>
                <td>
                    <TextField id="ros-url"
                        onChange={()=>{
                            setConnectionState('connect');
                            setUrl(document.getElementById("ros-url").value);
                        }}
                        defaultValue={url}
                        disabled={connectionState === "loading"}
                    />
                </td>
                <td>
                    <RosConnectButton 
                        state={connectionState} 
                        callback={connectEvent} 
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
                        checked={connected}
                        disabled 
                        onText="Connected" 
                        offText="Disconnected" 
                    />
                </td>
            </tr>
        </table>
    );
} 


