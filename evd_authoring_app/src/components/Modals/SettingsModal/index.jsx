import React, { useContext } from 'react';

import { Stack } from '@fluentui/react/lib/Stack';
import { Label } from '@fluentui/react/lib/Label';

import { RosSettings} from './RosSettings';
import { ModalWrapper } from '../ModalWrapper';
import { ApplicationSettings } from './ApplicationSettings';
import { ModalControlButtons } from '../ModalControlButtons';

import './index.css';

import { ModalContext } from '../../../contexts';


export const SettingsModal = (props) => {

    const {
        totalWidth
    } = props;

    const modalContext = useContext(ModalContext);

    return (
        <ModalWrapper
            name="settings"
            title="Settings"
            totalWidth={totalWidth}
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
                        callbacks={{'close': () => {modalContext.closeModal('settings')}}}
                        isPrimary={{'close': true}}
                    />
                </Stack.Item>

            </Stack>
        </ModalWrapper>
    );
}
