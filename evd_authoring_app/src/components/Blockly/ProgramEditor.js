import React, { Component } from 'react'
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

    componentDidMount = () => {
      window.setTimeout(() => {
        this.setState({
          toolboxCategories: this.state.toolboxCategories.concat([
            {
              name: 'Text2',
              blocks: [
                { type: 'text' },
                {
                  type: 'text_print',
                  values: {
                    TEXT: {
                      type: 'text',
                      shadow: true,
                      fields: {
                        TEXT: 'abc',
                      },
                    },
                  },
                },
              ],
            },
          ]),
        });
      }, 2000);
  
      window.setTimeout(() => {
        this.setState({
          toolboxCategories: [
            ...this.state.toolboxCategories.slice(0, this.state.toolboxCategories.length - 1),
            {
              ...this.state.toolboxCategories[this.state.toolboxCategories.length - 1],
              blocks: [
                { type: 'text' },
              ],
            },
          ],
        });
      }, 4000);
  
      window.setTimeout(() => {
        this.setState({
          toolboxCategories: this.state.toolboxCategories.slice(0, this.state.toolboxCategories.length - 1),
        });
      }, 10000);
    }

    workspaceDidChange = (workspace) => {

      console.log("In workspace did change callback");
      console.log(workspace);

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
    }

    render() { 
      return (
          <div id="blockly" style={{height: this.props.height, width: this.props.width, boxShadow: "3px 3px 3px #000"}}>
              <ReactBlockly.BlocklyEditor  
                  toolboxCategories={this.state.toolboxCategories}
                  workspaceConfiguration={{
                      grid: {
                      spacing: 20,
                      length: 3,
                      colour: '#ccc',
                      snap: true,
                      },
                  }}
                  initialXml={ConfigFiles.INITIAL_XML}
                  wrapperDivClassName="fill-height"
                  workspaceDidChange={this.workspaceDidChange}
              />
          </div>
          
      );
    }

}

export default ProgramEditor;