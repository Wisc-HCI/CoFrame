import React, { Component } from 'react';

// import Unity from 'react-unity-webgl';
import useGuiStore from '../../stores/GuiStore';
import {Scene} from 'robot-scene';

import { Tooltray } from './Tooltray';

import { 
    ThemeContext
} from "../../contexts";


export function Simulator(props) {

  const frame = useGuiStore(state => state.frame);

  return (
    <ThemeContext.Consumer>
        { themeValue => (
            <div style={{
                backgroundColor: themeValue.frameStyles.colors[frame],
                padding: 5
            }}>
                <Scene 
                    displayTfs={true}
                    displayGrid={true}
                    isPolar={false}
                    backgroundColor='#1e1e1e'
                    planeColor='#141414'
                    highlightColor='#ffffff'/>
                <div className="simulator-tooltray-container" >
                    <div className="simulator-tooltray">
                        <Tooltray 
                            active="rotate"
                        />
                    </div>
                </div>
            </div>
        )}
    </ThemeContext.Consumer>
);
}