import React, { Component } from 'react';
import ReactBlockly from 'react-blockly';
import Blockly from 'blockly';

import ConfigFiles from './content/programContent';
import parseWorkspaceXml from './BlocklyHelper';

import './index.css';

export class ProgramEditor extends Component {
  constructor(props) {
    super(props);
    this.state = {
      toolboxCategories: parseWorkspaceXml(ConfigFiles.INITIAL_TOOLBOX_XML),
    };
  }

  componentDidMount = () => {
    // TODO
  };

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
    const { toolboxCategories } = this.state;

    const padding = 5;
    const blocklyWidth = width - 2 * padding;
    const blocklyHeight = height - 2 * padding;

    return (
        <React.Fragment>
            <div
                id="blockly"
                style={{ height: `${blocklyHeight}px`, width: `${blocklyWidth}px` }}
            >
                <ReactBlockly.BlocklyEditor
                    toolboxCategories={toolboxCategories}
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
                    initialXml={ConfigFiles.INITIAL_XML}
                    wrapperDivClassName="fill-height"
                    workspaceDidChange={this.workspaceDidChange}
                />
            </div>
        </React.Fragment> 
    );
  }
}
