import React, {useState} from 'react';

import { TextField } from '@fluentui/react/lib/TextField';
import { Label } from '@fluentui/react/lib/Label';

import useApplicationStore from '../../../stores/ApplicationStore';


export function ApplicationSettings(props) {

    const {filename, setFilename} = useApplicationStore(state=({
        filename:state.filename,
        setFilename:state.setFilename
    }))

    return (
        <table className="settings-layout">
            <tr>
                <td className="settings-layout-first-cell">
                    <Label>Project Name:</Label>
                </td>
                <td>
                    <TextField id="settings-project-name"
                        onChange={(e) => setFilename(document.getElementById('settings-project-name').value)}
                        defaultValue={filename}
                    />
                </td>
            </tr>
        </table>
    );
}


