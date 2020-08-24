import React from 'react';
import { BrowserRouter as Router, Route } from 'react-router-dom';
import 'react-splitter-layout/lib/index.css';
import { Provider } from 'react-redux';
import store from './store';

import Header from './components/Header';
import Footer from './components/Footer';
import Simulator from './components/Simulator';
import ProgramEditor from './components/Blockly/ProgramEditor';
import TrajectoryEditor from './components/Blockly/TrajectoryEditor';
import WebAR from './components/WebAR';

class App extends React.Component {

  constructor(props) {
    super(props);

    this.state = {
      height: 0,
      width: 0,
      panelOpen: false
    };
    
    this.updateWindowDimensions = this.updateWindowDimensions.bind(this);
    this.setOpenPanel = this.setOpenPanel.bind(this);
  }

  componentWillMount() {
    this.updateWindowDimensions();
    window.addEventListener('resize', this.updateWindowDimensions);
  }

  componentDidMount() {
    this.updateWindowDimensions();
  }

  componentWillUnmount() {
    window.removeEventListener('resize', this.updateWindowDimensions);
  }

  updateWindowDimensions() {
    this.setState({...this.state, width: window.innerWidth, height: window.innerHeight});
  }

  setOpenPanel(state) {
    this.setState({...this.state, panelOpen: state});
  }
  
  render() {

    let totalHeight = this.state.height;
    console.log(`Height: ${totalHeight}`);
    let totalWidth = this.state.width;

    // handle header layout
    let headerHeight = Header.getDesiredHeight();
    if (headerHeight > totalHeight) {
      headerHeight = totalHeight;
    }
    totalHeight -= headerHeight;

    // handle program editor layout
    let programEditorHeight = totalHeight;
    let programEditorWidth = totalWidth;

    // handle side panel layout
    let sidePanelSize = 30; // %

    return (
      <Provider store={store}>
        <Router>
          <Route exact path="/" render={props => (
            <React.Fragment>
              <Header menuType="MAIN" width={'100%'} height={`${headerHeight}px`} />
              <ProgramEditor width={`${programEditorWidth}px`} height={`${programEditorHeight}px`} />
            </React.Fragment>
          )} />
          <Route path="/web_ar" component={props => (
            <React.Fragment>
              <Header menuType="WEB_AR" />
              <WebAR />
            </React.Fragment>
          )} />
        </Router>
      </Provider>
    );
  }
  
}

export default App;
