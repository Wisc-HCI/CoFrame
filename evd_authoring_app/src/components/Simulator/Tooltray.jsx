import React, { Fragment } from 'react';

import { useId } from '@uifabric/react-hooks';
import { FontIcon } from '@fluentui/react/lib/Icon';
import { TooltipHost, TooltipDelay, DirectionalHint } from '@fluentui/react';

import { ThemeContext } from "../../contexts";

import './index.css';


export const Tooltray = (props) => {

    const tooltipId = useId('simulator-tooltip');

    return (
        <ThemeContext.Consumer>
            { value => (
                <div 
                    style={{
                        background: value.theme.semanticColors.bodyBackground, 
                        boxShadow: '3px 3px 3px #000',
                        paddingLeft: '10px',
                        paddingRight: '9px',
                        paddingTop: '10px',
                        paddingBottom: '5px'
                    }}
                >
                    
                    <TooltipHost
                        delay={TooltipDelay.zero}
                        directionalHint={DirectionalHint.topCenter}
                        styles={{root: {display: 'inline-block'}}}
                        tooltipId={tooltipId}
                        tooltipProps={{
                            onRenderContent: () => (
                                <Fragment>
                                    <i>Camera Controls</i>
                                    <p>Left Click - Rotate</p>
                                    <p>Right Click - Pan</p>
                                    <p>Scroll - Zoom</p>
                                </Fragment>
                            )
                        }}
                    >
                        <FontIcon iconName="Help" styles={{root: { fontSize: 50 }}} />
                    </TooltipHost>

                </div>
            )}
        </ThemeContext.Consumer>
    );
};