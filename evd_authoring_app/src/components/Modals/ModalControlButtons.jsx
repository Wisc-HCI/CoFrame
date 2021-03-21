import React from 'react';

import { 
    Stack,
    PrimaryButton,
    DefaultButton
} from 'office-ui-fabric-react';


const nameLookup = {
    'close': 'Close',
    'cancel': 'Cancel',
    'submit': 'Submit'
}


export const ModalControlButtons = (props) => {

    const { order } = props;
    
    let {
        callbacks,
        isPrimary,
        disabled
    } = props;

    if (callbacks === undefined) {
        callbacks = {};
    } 

    if (isPrimary === undefined) {
        callbacks = {};
    }

    if (disabled === undefined) {
        disabled = {};
    }

    const btns = order.map(key => {
        
        let btnProps = { 
            key: key,
            onClick: callbacks[key],
            text: nameLookup[key],
            disabled: (disabled[key] !== undefined) ? disabled[key] : false
        };

        let btn = null;
        if (isPrimary[key]) {
            btn = (<PrimaryButton {...btnProps} />);
        } else {
            btn = (<DefaultButton {...btnProps} />);
        }

        return btn;
    });

    return (
        <Stack horizontal tokens={{ childrenGap: '10' }}>
            {btns}
        </Stack>
    );
};