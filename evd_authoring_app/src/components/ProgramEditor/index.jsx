import React, { Component } from 'react';
// import ReactBlockly from 'react-blockly';

import Blockly from 'node-blockly/browser';
import BlocklyDrawer, { Block, Category } from 'react-blockly-drawer';

import { 
    evdScriptBlocklyToolbox, 
    evdScriptBlocklyInitialXML 
} from '../../model/evdScript/blockly.js';

import './index.css';


export class ProgramEditor extends Component {
  constructor(props) {
    super(props);

    this.state = {
      toolbox: evdScriptBlocklyToolbox(),
      initialXml: evdScriptBlocklyInitialXML()
    };
  }

  workspaceDidChange = (workspace) => {
    /*
      workspace.registerButtonCallback('myFirstButtonPressed', () => {
          alert('button is pressed');
      });
      */

    /*
      const newXml = Blockly.Xml.domToText(Blockly.Xml.workspaceToDom(workspace));
      document.getElementById('generated-xml').innerText = newXml;
      */

    /*
      const code = Blockly.JavaScript.workspaceToCode(workspace);
      document.getElementById('code').value = code;
      */
  };

  render() {
    const { 
        toolbox, 
        initialXml 
    } = this.state;
  
    return (
        <div
            id="blockly"
            style={{ 
                height: '100%', 
                width: '100%' 
            }}
        >
        <BlocklyDrawer
          tools={toolbox}
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
        </div>
    );
  }
}
