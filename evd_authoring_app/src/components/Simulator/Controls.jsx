import React, { Component } from 'react';

import { IconButton } from 'office-ui-fabric-react';

import styles from '../../frameStyles';

class Controls extends Component {

  constructor(props) {
    super(props);

    this.onPauseClicked = this.onPauseClicked.bind(this);
    this.onPlayClicked = this.onPlayClicked.bind(this);
    this.onResetClicked = this.onResetClicked.bind(this);
  }

  onPauseClicked() {
    console.log('On pause clicked');
  }

  onPlayClicked() {
    console.log('On play clicked');
  }

  onResetClicked() {
    console.log('On reset clicked');
  }

  render() {

    const { frame } = this.props;

    return (
        <div style={{paddingLeft: '10px', paddingRight: '10px'}}>
            <i>Controls</i>

            <br />

            <IconButton
                iconProps={{ iconName: 'Refresh' }}
                title="Reset"
                ariaLabel="Reset"
                onClick={this.onResetClicked}
            />
            <IconButton
                iconProps={{ iconName: 'Play' }}
                title="Play"
                ariaLabel="Play"
                onClick={this.onPlayClicked}
            />
            <IconButton
                iconProps={{ iconName: 'Pause' }}
                title="Pause"
                ariaLabel="Pause"
                onClick={this.onPauseClicked}
            />
        </div>
    );
  }
}

export default Controls;
