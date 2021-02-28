import React, { Component } from 'react';

import Unity, { UnityContent } from 'react-unity-webgl';
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
    const { frame, width, height } = this.props;
    const frameColor = styles.colors[frame];

    return (
        <div
            style={{
                padding: '5px',
                backgroundColor: frameColor,
                width: width,
                height: height
            }}
        >
            <Unity unityContent={this.unityContent} />
        </div>
    );
  }
}