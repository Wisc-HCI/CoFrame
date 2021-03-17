import React from 'react';

import { Stack } from 'office-ui-fabric-react';

import { RosSettings} from './RosSettings';
import { ModalWrapper } from '../ModalWrapper';
import { ApplicationSettings } from './ApplicationSettings';
import { ModalControlButtons } from '../ModalControlButtons';


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
