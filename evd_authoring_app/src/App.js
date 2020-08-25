import React from 'react';
import { BrowserRouter as Router, Route } from 'react-router-dom';
import { Provider } from 'react-redux';
import store from './store';

import Header from './components/Header';
//import Footer from './components/Footer';
//import Simulator from './components/Simulator';
import ProgramEditor from './components/Blockly/ProgramEditor';
//import TrajectoryEditor from './components/Blockly/TrajectoryEditor';
import DetailPanel from './components/DetailPanel';

import WebAR from './components/WebAR';

import ParametersPanel from './components/DetailPanel/ParametersPanel';
import { LoremIpsum } from 'react-lorem-ipsum';
import { ScrollablePane } from 'office-ui-fabric-react/lib/ScrollablePane';


import { Stack } from 'office-ui-fabric-react';

class App extends React.Component {

  constructor(props) {
    super(props);

    this.state = {
      height: 0,
      width: 0
    };
    
    this.updateWindowDimensions = this.updateWindowDimensions.bind(this);
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

  render() {

    let distanceFromTop = 0;

    let totalHeight = this.state.height;
    console.log(`Height: ${totalHeight}`);
    let totalWidth = this.state.width;

    // handle header layout
    let headerHeight = Header.getDesiredHeight();
    if (headerHeight > totalHeight) {
      headerHeight = totalHeight;
    }
    totalHeight -= headerHeight;
    distanceFromTop += headerHeight;

    // handle detail panel
    let detailWidth = DetailPanel.getDesiredWidth();
    if (detailWidth > totalWidth) {
      detailWidth = totalWidth;
    }
    totalWidth -= detailWidth;

    // handle program editor layout
    let programEditorHeight = totalHeight;
    let programEditorWidth = totalWidth;

    return (
      <Provider store={store}>
        <Router>
          <Route exact path="/" render={props => (
            <React.Fragment>
              <Header theme={this.props.theme} width={'100%'} height={`${headerHeight}px`} filename={"Untitled"}/>
              
              <div style={{width: `${programEditorWidth + detailWidth}px`}}>
                <Stack horizontal>
                  <ProgramEditor theme={this.props.theme} width={`${programEditorWidth}px`} height={`${programEditorHeight}px`} />
                  <DetailPanel theme={this.props.theme} width={detailWidth} height={programEditorHeight} distanceFromTop={distanceFromTop}/>
                </Stack>
              </div>
              
            </React.Fragment>
          )} />
          <Route path="/web_ar" component={props => (
            <WebAR />
          )} />
        </Router>
      </Provider>
    );
  }
  
}

export default App;
