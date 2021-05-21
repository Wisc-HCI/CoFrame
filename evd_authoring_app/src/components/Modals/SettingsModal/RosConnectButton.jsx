import React from 'react';

import { DefaultButton, PrimaryButton } from '@fluentui/react/lib/Button';


export const RosConnectButton = (props) => {

    const { state, callback } = props;

    let button = null;
    switch (state) {
        case "refresh":
            button = (<DefaultButton text="Refresh" onClick={callback} />);
            break;
        case "connect":
            button = (<PrimaryButton text="Connect" onClick={callback} />);
            break;
        case "loading":
            button = (<DefaultButton text="Connecting..." disabled />);
            break;
        default:
            break;
    }

    return (
        <React.Fragment>
            {button}
        </React.Fragment>
    );
};