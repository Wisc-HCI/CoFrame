import React, { Component } from 'react';
import ReactBlockly from 'react-blockly';
import Blockly from 'blockly';

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
    const { width, height } = this.props;
    const { toolbox, initialXml } = this.state;

    const padding = 5;
    const blocklyWidth = width - 2 * padding;
    const blocklyHeight = height - 2 * padding;

    console.log(Blockly);

    return (
        <div
            id="blockly"
            style={{ height: `${blocklyHeight}px`, width: `${blocklyWidth}px` }}
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
