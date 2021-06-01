import React, { Component } from 'react';
// import ReactBlockly from 'react-blockly';

// import { 
//     evdScriptBlocklyToolbox, 
//     evdScriptBlocklyInitialXML 
// } from '../../model/evdScript';

import './index.css';


export class ProgramEditor extends Component {
  // constructor(props) {
  //   super(props);

    // this.state = {
    //   toolbox: evdScriptBlocklyToolbox(),
    //   initialXml: evdScriptBlocklyInitialXML()
    // };
  // }

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
    // const { 
    //     toolbox, 
    //     initialXml 
    // } = this.state;
    
    return (
      <div 
        id="blockly"
        style={{ 
          height: '100%', 
          width: '100%'
      }}>
        BLOCKLY HERE
      </div>
    )
  }
}
