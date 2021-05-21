import React, { Fragment, useContext } from 'react';

import { DefaultButton, PrimaryButton } from '@fluentui/react/lib/Button';

import { ThemeContext } from "../../contexts";


export const StyledButton = (props) => {

    const { 
        primary, 
        callback, 
        frame, 
        text
    } = props;

    const theme = useContext(ThemeContext);
    const frameColor = theme.frameStyles.colors[frame];

    let styles = null;
    if (primary) {
        styles = {
            root: {
                backgroundColor: frameColor,
                borderColor: frameColor,
                borderRadius: '0'
            },
            rootHovered: {
                backgroundColor: frameColor,
                borderColor: frameColor
            },
            rootPressed: {
                backgroundColor: frameColor,
                borderColor: frameColor
            }
        }
    } else {
        styles = {
            root: {
                color: frameColor,
                borderRadius: '0'
            }
        }
    }

    const btnProps = {
        text: text,
        onClick: callback,
        styles: styles
    };

    return (
        <Fragment>
            {(primary) ? <PrimaryButton {...btnProps} /> : <DefaultButton {...btnProps} />}
        </Fragment>
    );
}