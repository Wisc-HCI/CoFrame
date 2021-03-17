import React from 'react';

import { 
    PrimaryButton, 
    DefaultButton, 
    Stack, 
    TextField,
    Label,
    Toggle,
    Spinner, 
    SpinnerSize
} from 'office-ui-fabric-react';

import { ModalControlButtons } from './ModalControlButtons';
import { ModalWrapper } from './ModalWrapper';


const RosConnectButton = (props) => {

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

const RosSettings = (props) => {

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

const ApplicationSettings = (props) => {

    const { textCallback } = props;

    return (
        <React.Fragment>
            <Stack horizontal tokens={{ childrenGap: '20' }}>

                <Label>Project Name:</Label>

                <TextField 
                    onChange={textCallback}
                    defaultValue={'Untitled'}
                />

            </Stack>
        </React.Fragment>
    );
};


export class SettingsModal extends React.Component {

    render() {
        const { 
            open,
            totalWidth,
            closeModal
        } = this.props;

        const width = totalWidth / 2;

        return (
            <ModalWrapper
                open={open}
                title="Settings"
                hideModal={() => {closeModal('settings')}}
                width={width}
            >
                <Stack>

                    <br />
                    <Stack.Item>
                        <ApplicationSettings />
                    </Stack.Item>

                    <br />
                    <Stack.Item>
                        <RosSettings state="loading" />
                    </Stack.Item>

                    <br />
                    <Stack.Item align="center">
                        <ModalControlButtons 
                            order={['close']} 
                            callbacks={{'close': () => {closeModal('settings')}}}
                            isPrimary={{'close': true}}
                        />
                    </Stack.Item>

                </Stack>
            </ModalWrapper>
        );
    }
}
