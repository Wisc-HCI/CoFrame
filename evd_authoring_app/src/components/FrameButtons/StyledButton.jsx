import React from 'react';

import {
    DefaultButton,
    PrimaryButton,
} from 'office-ui-fabric-react';

import { ThemeContext } from "../../contexts";


export class StyledButton extends React.Component {

    static contextType = ThemeContext;

    render() {
        const { 
            primary, 
            callback, 
            frame, 
            text
        } = this.props;

        const frameColor = this.context.frameStyles.colors[frame];

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
        <React.Fragment>
            {(primary) ? <PrimaryButton {...btnProps} /> : <DefaultButton {...btnProps} />}
        </React.Fragment>
    );
}
}