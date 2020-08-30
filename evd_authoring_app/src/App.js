import React from 'react';
import { BrowserRouter as Router, Route } from 'react-router-dom';

import { Stack } from 'office-ui-fabric-react';

import { Provider } from 'react-redux';
import store from './store';

import Header from './components/Header';
import Simulator from './components/Simulator';
import ProgramEditor from './components/Blockly/ProgramEditor';
import SettingsModal from './components/Modals/SettingsModal';
import UploadModal from './components/Modals/UploadModal';
import DownloadModal from './components/Modals/DownloadModal';
import OpenModal from './components/Modals/OpenModel';

class App extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      height: 0,
      width: 0,
      downloadModalOpen: false,
      uploadModalOpen: false,
      openModalOpen: false,
      settingsModalOpen: false,
    };

    this.updateWindowDimensions = this.updateWindowDimensions.bind(this);
    this.closeModal = this.closeModal.bind(this);
    this.onHeaderButtonClicked = this.onHeaderButtonClicked.bind(this);
    this.computeHeaderLayout = this.computeHeaderLayout.bind(this);
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
    this.setState((prevState) => {
      return {
        ...prevState,
        width: window.innerWidth,
        height: window.innerHeight,
      };
    });
  }

  closeModal(modal) {
    this.setState((prevState) => {
      return {
        ...prevState,
        downloadModalOpen:
          modal === 'download' ? false : prevState.downloadModalOpen,
        uploadModalOpen: modal === 'upload' ? false : prevState.uploadModalOpen,
        openModalOpen: modal === 'open' ? false : prevState.openModalOpen,
        settingsModalOpen:
          modal === 'settings' ? false : prevState.settingsModalOpen,
      };
    });
  }

  onHeaderButtonClicked(button) {
    this.setState((prevState) => {
      return {
        ...prevState,
        downloadModalOpen: button === 'download',
        uploadModalOpen: button === 'upload',
        openModalOpen: button === 'open',
        settingsModalOpen: button === 'settings',
      };
    });
  }

  computeHeaderLayout(layoutObj) {
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

  computeMainLayout(layoutObj) {
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

  render() {
    const {
      width,
      height,
      downloadModalOpen,
      uploadModalOpen,
      openModalOpen,
      settingsModalOpen,
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

    layoutObj = this.computeHeaderLayout(layoutObj);
    layoutObj = this.computeMainLayout(layoutObj);

    return (
      <Provider store={store}>
        <Router>
          <Route
            exact
            path="/"
            render={() => (
              <>
                <Header
                  theme={theme}
                  width={layoutObj.headerWidth}
                  height={layoutObj.headerHeight}
                  filename="Untitled"
                  onButtonClick={this.onHeaderButtonClicked}
                />

                <div
                  style={{
                    width: `${layoutObj.mainWidth}px`,
                    height: `${layoutObj.mainHeight}px`,
                  }}
                >
                  <Stack horizontal>
                    <div
                      style={{
                        paddingTop: `${mainPadding}px`,
                        paddingBottom: `${mainPadding}px`,
                        paddingRight: `${mainPadding / 2}px`,
                      }}
                    >
                      <ProgramEditor
                        theme={theme}
                        width={layoutObj.progamWidth - mainPadding / 2}
                        height={layoutObj.programHeight - mainPadding}
                      />
                    </div>

                    <div
                      style={{
                        paddingLeft: `${mainPadding / 2}px`,
                        paddingTop: `${mainPadding}px`,
                        paddingBottom: `${mainPadding}px`,
                      }}
                    >
                      <Simulator
                        theme={theme}
                        width={layoutObj.simulatorWidth - mainPadding / 2}
                        height={layoutObj.simulatorHeight - mainPadding}
                      />
                    </div>
                  </Stack>
                </div>

                <DownloadModal
                  open={downloadModalOpen}
                  closeModal={this.closeModal}
                />
                <UploadModal
                  open={uploadModalOpen}
                  closeModal={this.closeModal}
                />
                <OpenModal 
                  open={openModalOpen} 
                  closeModal={this.closeModal} 
                />
                <SettingsModal
                  open={settingsModalOpen}
                  closeModal={this.closeModal}
                />
              </>
            )}
          />
        </Router>
      </Provider>
    );
  }
}

export default App;
