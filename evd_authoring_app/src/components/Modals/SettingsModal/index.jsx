import React from 'react';

import { 
    Label,
    Stack 
} from 'office-ui-fabric-react';

import { RosSettings} from './RosSettings';
import { ModalWrapper } from '../ModalWrapper';
import { ApplicationSettings } from './ApplicationSettings';
import { ModalControlButtons } from '../ModalControlButtons';

import './index.css';


export const SettingsModal = (props) => {

    const { 
        open,
        totalWidth,
        closeModal
    } = props;

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
                    <Label>Application</Label>
                    <ApplicationSettings />
                </Stack.Item>

                <br />
                <Stack.Item>
                    <Label>ROS</Label>
                    <RosSettings />
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
