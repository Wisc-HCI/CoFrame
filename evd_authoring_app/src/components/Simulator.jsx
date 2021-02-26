import React, { Component } from 'react';

import { Separator } from 'office-ui-fabric-react';

import Unity, { UnityContent } from 'react-unity-webgl';
import Controls from './Controls';
import styles from '../frameStyles';

export class Simulator extends Component {

  constructor(props) {
    super(props);

    this.unityContent = new UnityContent(
        './simulator/Build/Build.json',
        './simulator/Build/UnityLoader.js',
        {
            adjustOnWindowResize: true,
        },
    );    
  }

  render() {
    const { frame } = this.props;
    const frameColor = styles.colors[frame];

    return (
        <React.Fragment>
              
            <div
                style={{
                    padding: '5px',
                    backgroundColor: frameColor,
                }}
            >
                <Unity unityContent={this.unityContent} />
            </div>

            <Separator />

            <Controls frame={frame}/>

        </React.Fragment>
    );
  }
}