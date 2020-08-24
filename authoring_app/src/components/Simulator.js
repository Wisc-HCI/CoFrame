import React, { Component } from 'react'
import Unity, { UnityContent } from "react-unity-webgl";

class Simulator extends Component {
    constructor(props) {
        super(props);

        this.unityContent = new UnityContent(
            "./simulator/Build/Build.json",
            "./simulator/Build/UnityLoader.js",
            {
              adjustOnWindowResize: true 
            }
          );
    }
    
    render() {
        return (
            <Unity unityContent={this.unityContent} />
        );
    }
}

export default Simulator;
