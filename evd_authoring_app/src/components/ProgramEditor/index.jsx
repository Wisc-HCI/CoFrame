import React, { Component } from 'react';
// import ReactBlockly from 'react-blockly';

import Blockly from 'node-blockly/browser';
import BlocklyDrawer, { Block, Category } from 'react-blockly-drawer';

import { 
    evdScriptBlocklyToolbox, 
    evdScriptBlocklyInitialXML 
} from '../../model/evdScript/blockly.js';

import './index.css';

// We might want to look into the ResizeObserver for updating
// the size of this component more gracefully. Right now it
// initializes too large, and if the screen is resized it breaks.
// import { ResizeObserver } from "@juggle/resize-observer";


export class ProgramEditor extends Component {
  constructor(props) {
    super(props);

    this.state = {
      toolbox: evdScriptBlocklyToolbox(),
      initialXml: evdScriptBlocklyInitialXML()
    };
  }

  render() {
    const toolbox = this.state.toolbox;
    var initialXml = this.state.initialXml;
  
    return (
        
        <BlocklyDrawer
          style={{ 
            width:'100%',
            height:'100%'
          }}
          tools={toolbox}
          onChange={(code, workspace) => {
            initialXml = workspace;
            console.log(initialXml)
          }}
          workspaceXML={initialXml}
          language={Blockly.JavaScript}
          appearance={
            {
              categories: {
                Testing: {
                  colour: '360'
                },
                Machine: {
                  colour: '50'
                },
                Location: {
                  colour: '260'
                },
                Skills: {
                  colour: '210'
                }
              },
            }
          }
        >
        </BlocklyDrawer>
    );
  }
}
