import React from 'react';
import { BrowserRouter as Router, Route } from 'react-router-dom';

import Header from './components/Header';

import Body from './components/Body';
import ProgramEditor from './components/Blockly/ProgramEditor';
import SettingsModal from './components/Modals/SettingsModal';
import UploadModal from './components/Modals/UploadModal';
import DownloadModal from './components/Modals/DownloadModal';
import OpenModal from './components/Modals/OpenModel';

class App extends React.Component {

  static computeHeaderLayout(layoutObj) {

    const newLayoutObj = { ...layoutObj };

    // handle header layout
    newLayoutObj.headerHeight = Header.getDesiredHeight();
    if (newLayoutObj.headerHeight > newLayoutObj.totalHeight) {
      newLayoutObj.headerHeight = newLayoutObj.totalHeight;
    }
    newLayoutObj.distanceFromTop += newLayoutObj.headerHeight;
    newLayoutObj.remainingHeight -= newLayoutObj.headerHeight;

    newLayoutObj.headerWidth = newLayoutObj.totalWidth;

    newLayoutObj.mainHeight = newLayoutObj.remainingHeight;
    newLayoutObj.mainWidth = newLayoutObj.totalWidth;

    return newLayoutObj;
  }

  static computeMainLayout(layoutObj) {
    const newLayoutObj = { ...layoutObj };

    let remainingWidth = newLayoutObj.totalWidth;

    // handle program layout
    newLayoutObj.programHeight = newLayoutObj.remainingHeight;
    newLayoutObj.progamWidth = ProgramEditor.getDesiredWidth();
    if (newLayoutObj.progamWidth > newLayoutObj.totalWidth) {
      newLayoutObj.progamWidth = newLayoutObj.totalWidth;
    }
    remainingWidth -= newLayoutObj.progamWidth;

    // handle simulator layout
    newLayoutObj.simulatorWidth = remainingWidth;
    newLayoutObj.simulatorHeight = newLayoutObj.remainingHeight;

    return newLayoutObj;
  }

  constructor(props) {
    super(props);

    this.state = {
      height: 0,
      width: 0,

      downloadModalOpen: false,
      uploadModalOpen: false,
      openModalOpen: false,
      settingsModalOpen: false,

      filename: 'Untitled',
      model: null
    };

    this.updateWindowDimensions = this.updateWindowDimensions.bind(this);
    this.closeModal = this.closeModal.bind(this);
    this.onHeaderButtonClicked = this.onHeaderButtonClicked.bind(this);
  }

  UNSAFE_componentWillMount() {
    this.updateWindowDimensions();
    window.addEventListener('resize', this.updateWindowDimensions);
  }

  componentDidMount() {
    this.updateWindowDimensions();
  }

  componentWillUnmount() {
    window.removeEventListener('resize', this.updateWindowDimensions);
  }

  onHeaderButtonClicked(button) {
    this.setState((prevState) => {
      return {
        downloadModalOpen: button === 'download',
        uploadModalOpen: button === 'upload',
        openModalOpen: button === 'open',
        settingsModalOpen: button === 'settings',
      };
    });
  }

  closeModal(modal) {
    this.setState((prevState) => {
      return {
        downloadModalOpen: modal === 'download' ? false : prevState.downloadModalOpen,
        uploadModalOpen: modal === 'upload' ? false : prevState.uploadModalOpen,
        openModalOpen: modal === 'open' ? false : prevState.openModalOpen,
        settingsModalOpen: modal === 'settings' ? false : prevState.settingsModalOpen,
      };
    });
  }

  updateWindowDimensions() {
    this.setState((prevState) => {
      return {
        width: window.innerWidth,
        height: window.innerHeight,
      };
    });
  }

  render() {

    const {
      width,
      height,
      downloadModalOpen,
      uploadModalOpen,
      openModalOpen,
      settingsModalOpen,
      filename
    } = this.state;

    const { theme } = this.props;

    let layoutObj = {
      distanceFromTop: 0,
      remainingHeight: height,
      totalHeight: height,
      totalWidth: width,
      headerHeight: 0,
      headerWidth: 0,
      mainHeight: 0,
      mainWidth: 0,
      programHeight: 0,
      programWidth: 0,
      simulatorHeight: 0,
      simulatorWidth: 0,
    };

    const mainPadding = 10;

    layoutObj = App.computeHeaderLayout(layoutObj);
    layoutObj = App.computeMainLayout(layoutObj);

    return (
      <Router>
        <Route
          exact
          path="/"
          render={() => (
            <React.Fragment>

              <Header
                theme={theme}
                width={layoutObj.headerWidth}
                height={layoutObj.headerHeight}
                filename={filename}
                onButtonClick={this.onHeaderButtonClicked}
              />

              <Body 
                layoutObj={layoutObj}
                theme={theme}
                mainPadding={mainPadding}
              />

              <DownloadModal
                open={downloadModalOpen}
                closeModal={this.closeModal}
                theme={theme}
                totalWidth={layoutObj.totalWidth}
              />
              <UploadModal
                open={uploadModalOpen}
                closeModal={this.closeModal}
                theme={theme}
                totalWidth={layoutObj.totalWidth}
              />
              <OpenModal
                open={openModalOpen}
                closeModal={this.closeModal}
                theme={theme}
                totalWidth={layoutObj.totalWidth}
              />
              <SettingsModal
                open={settingsModalOpen}
                closeModal={this.closeModal}
                theme={theme}
                totalWidth={layoutObj.totalWidth}
              />

            </React.Fragment>
          )}
        />
      </Router>
    );
  }
}

export default App;
