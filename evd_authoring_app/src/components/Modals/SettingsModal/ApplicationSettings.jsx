import React from 'react';

import { TextField } from '@fluentui/react/lib/TextField';
import { Label } from '@fluentui/react/lib/Label';

import { ApplicationContext } from '../../../contexts';


export class ApplicationSettings extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            timeout: null
        }

        this.textChanged = this.textChanged.bind(this);
    }

    textChanged(service) {

        if (this.state.timeout !== null) {
            clearTimeout(this.state.timeout);
        }

        this.setState({
            timeout: setTimeout(() => {
                service.filename = document.getElementById('settings-project-name').value;
                this.setState({ timeout: null });
            }, 500)
        })
    }

    render() {

        return (
            <ApplicationContext.Consumer>
                { appValue => (
                    
                    <table className="settings-layout">
                        <tr>
                            <td className="settings-layout-first-cell">
                                <Label>Project Name:</Label>
                            </td>
                            <td>
                                <TextField id="settings-project-name"
                                    onChange={(e) => {this.textChanged(appValue.service)}}
                                    defaultValue={appValue.filename}
                                />
                            </td>
                        </tr>
                    </table>
    
                )}
            </ApplicationContext.Consumer>
        );
    }
}


