import React from 'react';

import Unity, { UnityContent } from 'react-unity-webgl';

import { Tooltray } from './Tooltray';

import { 
    ThemeContext,
    FrameContext,
    ControlsContext
} from "../../contexts";


export class Simulator extends React.Component {

  constructor(props) {
    super(props);

    this.unityContent = new UnityContent(
        './simulator/Build/Build.json',
        './simulator/Build/UnityLoader.js',
        {
            adjustOnWindowResize: true,
        }
    );
    
    this.recaptureInputAndFocus = this.recaptureInputAndFocus.bind(this);
    this.controlSelected = this.controlSelected.bind(this);
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

  controlSelected() {
      // TODO hook into unity control state
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
                        <ControlsContext.Consumer>
                            { controlsValue => (
                                <div
                                    style={{
                                        padding: '5px',
                                        backgroundColor: controlsValue.inSetup ? undefined : themeValue.frameStyles.colors[frameValue.frame],
                                        width: width,
                                        height: height
                                    }}
                                >
                                    <Unity unityContent={this.unityContent}  />
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
                        </ControlsContext.Consumer> 
                    )}
                </FrameContext.Consumer>
            )}
        </ThemeContext.Consumer>
    );
  }
}