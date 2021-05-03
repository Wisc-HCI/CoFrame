import React, { Component } from 'react';

import Unity from 'react-unity-webgl';

import { Tooltray } from './Tooltray';

import { 
    ThemeContext,
    FrameContext,
    UnityContext
} from "../../contexts";


export class Simulator extends Component {

  constructor(props) {
    super(props);
    
    this.recaptureInputAndFocus = this.recaptureInputAndFocus.bind(this);
  }

  recaptureInputAndFocus() {
    const canvas = document.getElementById("#canvas");
    if(canvas) {
        canvas.setAttribute("tabindex", "1");
        canvas.focus(); 
    } else {
        setTimeout(this.recaptureInputAndFocus, 100);
    }
  }

  onComponentDidMount() {
    this.recaptureInputAndFocus();
  }

  render() {

    const { 
        width, 
        height 
    } = this.props;



    return (
        <ThemeContext.Consumer>
            { themeValue => (
                <FrameContext.Consumer>
                    { frameValue => (
                        <UnityContext.Consumer>
                            { unityValue => (
                                <div
                                    style={{
                                        padding: '5px',
                                        backgroundColor: themeValue.frameStyles.colors[frameValue.frame],
                                        width: width,
                                        height: height
                                    }}
                                >
                                    <Unity unityContent={unityValue.simulator}  />
                                    <div className="simulator-tooltray-container" >
                                        <div className="simulator-tooltray">
                                            <Tooltray 
                                                active="rotate" 
                                                callback={this.controlSelected}
                                            />
                                        </div>
                                    </div>    
                                </div> 
                            )}
                        </UnityContext.Consumer>
                    )}
                </FrameContext.Consumer>
            )}
        </ThemeContext.Consumer>
    );
  }
}