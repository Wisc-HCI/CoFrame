import React from 'react';

import { 
    PrimaryButton, 
    DefaultButton
} from 'office-ui-fabric-react';


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