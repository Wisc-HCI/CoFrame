import React from 'react';

import { IconButton } from 'office-ui-fabric-react';


export class Controls extends React.Component {

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

    const { 
        width, 
        height 
    } = this.props;

    return (
        <div 
            style={{
                paddingLeft: '10px', 
                paddingRight: '10px',
                width: width,
                height: height
            }}
        >
            <i style={{fontSize: '18px'}}>Controls</i>

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
