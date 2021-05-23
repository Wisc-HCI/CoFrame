import React, { Component } from 'react';
// import ReactBlockly from 'react-blockly';

import { 
    evdScriptBlocklyToolbox, 
    evdScriptBlocklyInitialXML 
} from '../../model/evdScript';

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

    const padding = 5;
    
    return (
      <div 
        id="blockly"
        style={{ 
          height: 'calc(100vh-64px)', 
          width: '45vw'
      }}/>
    )

    return (
        <div
            id="blockly"
            style={{ 
              height: 'calc(100vh-64px)', 
              width: '45vw'
            }}
        >
            <ReactBlockly.BlocklyEditor
                toolboxCategories={toolbox}
                workspaceConfiguration={{
                    grid: {
                        spacing: 20,
                        length: 3,
                        colour: '#ccc',
                        snap: true,
                    },
                    scrollbars: true,
                    trashcan: false,
                    renderer: 'thrasos',
                }}
                initialXml={initialXml}
                wrapperDivClassName="fill-height"
                workspaceDidChange={this.workspaceDidChange}
            />
        </div>
    );
  }
}
