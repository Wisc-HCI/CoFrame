import React, { Component } from 'react';
import ReactBlockly from 'react-blockly';
import Blockly from 'blockly';

import ConfigFiles from './content/programContent';
import parseWorkspaceXml from './BlocklyHelper';

import './styles.css';

class ProgramEditor extends Component {
  constructor(props) {
    super(props);
    this.state = {
      toolboxCategories: parseWorkspaceXml(ConfigFiles.INITIAL_TOOLBOX_XML),
    };
  }

  static getDesiredHeight() {
    return null;
  }

  static getDesiredWidth() {
    return 600; // px
  }

  componentDidMount = () => {

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
    const { theme, width, height } = this.props;
    const { toolboxCategories } = this.state;

    const padding = 5;

    const outerWidth = width;
    const outerHeight = height;
    const innerWidth = outerWidth - 2 * padding;
    const innerHeight = outerHeight - 2 * padding;
    const blocklyWidth = innerWidth;
    const blocklyHeight = innerHeight;

    return (
      <div
        style={{
          padding: `${padding}px`,
          backgroundColor: theme.semanticColors.bodyBackground,
          boxShadow: '3px 3px 3px #000',
          height: `${innerHeight}px`,
          width: `${innerWidth}px`,
        }}
      >
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
              scrollbars: false,
              trashcan: false,
              renderer: 'thrasos',
            }}
            initialXml={ConfigFiles.INITIAL_XML}
            wrapperDivClassName="fill-height"
            workspaceDidChange={this.workspaceDidChange}
          />
        </div>
      </div>
    );
  }
}

export default ProgramEditor;
