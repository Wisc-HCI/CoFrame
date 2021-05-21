import React, { useState, useContext }  from 'react';

import { saveAs } from 'file-saver';

import { Stack } from '@fluentui/react/lib/Stack';

import { ModalWrapper } from './ModalWrapper';
import { ModalControlButtons } from './ModalControlButtons';

import { ApplicationContext, EvDScriptContext, ModalContext } from '../../contexts';


export const DownloadModal = (props) => {

    const { 
        totalWidth
    } = props;

    const [downloading, setDownloading] = useState(false);
    
    const appContext = useContext(ApplicationContext);
    const evdContext = useContext(EvDScriptContext);
    const modalContext = useContext(ModalContext);

    if (modalContext.state['download'] && !downloading) {

        const program = evdContext.program;
        const text = JSON.stringify((program === null) ? {} : program.toDict());

        const blob = new Blob([text], { type: 'text/plain;charset=utf-8' });
        saveAs(blob, `${appContext.filename}.json`);
        setDownloading(true);
    }

    const closeFnt = () => {
        modalContext.closeModal('download');
        setDownloading(false);
    };

    return (
        <ModalWrapper
            name="download"
            title="Download"
            totalWidth={totalWidth}
            closeCb={closeFnt}
        >
            <Stack>
                <br />

                <Stack.Item align="center">
                    <p>File is downloading. Press the <i><b>Close</b></i> button to return to design.</p>
                </Stack.Item>

                <br />

                <Stack.Item align="center">
                    <ModalControlButtons 
                        order={['close']} 
                        callbacks={{'close': closeFnt}}
                        isPrimary={{'close': true}}
                    />
                </Stack.Item>
            </Stack>
        </ModalWrapper>
    );
}