import React from 'react';

import { 
    Stack, 
    TextField,
    Label,
    Toggle,
    Spinner, 
    SpinnerSize
} from 'office-ui-fabric-react';


export const RosSettings = (props) => {

    const { state, textCallback, btnCallback } = props;

    let spinner = null;
    if (state === "loading") {
        spinner = (<Spinner size={SpinnerSize.medium} />);
    }

    return (
        <React.Fragment>
            <Stack horizontal tokens={{ childrenGap: '20' }}>

                <Label>ROS Server:</Label>

                <TextField 
                    prefix="ws://"
                    onChange={textCallback}
                    defaultValue={'localhost:9090'}
                    disabled={state === "loading"}
                />

                <RosConnectButton state={state} callback={btnCallback} />

                {spinner}
                
            </Stack>

            <br />

            <Stack horizontal tokens={{ childrenGap: '20' }} style={{paddingLeft: '2rem'}}>
                <Label>Status:</Label>
                <Toggle disabled onText="Connected" offText="Disconnected" />
            </Stack>

            <Stack horizontal tokens={{ childrenGap: '20' }} style={{paddingLeft: '2rem'}}>
                <Label>Root Frame:</Label>

                <TextField 
                    defaultValue={'app'}
                    disabled
                />
            </Stack>
           
        </React.Fragment>
    );
};