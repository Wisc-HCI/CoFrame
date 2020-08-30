import React, { Component } from 'react';
import Unity, { UnityContent } from 'react-unity-webgl';

import {
  Stack,
  IconButton,
  DefaultButton,
  PrimaryButton,
} from 'office-ui-fabric-react';

const SAFETY_COLOR = '#54eb61';
const QUALITY_COLOR = '#6c8feb';
const PERFORMANCE_COLOR = '#ebbc49';
const BUISNESS_COLOR = '#eb3d7a';

class Simulator extends Component {
  constructor(props) {
    super(props);

    this.unityContent = new UnityContent(
      './simulator/Build/Build.json',
      './simulator/Build/UnityLoader.js',
      {
        adjustOnWindowResize: true,
      },
    );

    this.state = {
      frame: 'quality',
    };

    this.onSafetyClicked = this.onSafetyClicked.bind(this);
    this.onQualityClicked = this.onQualityClicked.bind(this);
    this.onPerformanceClicked = this.onPerformanceClicked.bind(this);
    this.onBuisnessClicked = this.onBuisnessClicked.bind(this);

    this.onPauseClicked = this.onPauseClicked.bind(this);
    this.onPlayClicked = this.onPlayClicked.bind(this);
    this.onResetClicked = this.onResetClicked.bind(this);
  }

  onSafetyClicked() {
    this.setState((prevState) => {
      return { ...prevState, frame: 'safety' };
    });
  }

  onQualityClicked() {
    this.setState((prevState) => {
      return { ...prevState, frame: 'quslity' };
    });
  }

  onPerformanceClicked() {
    this.setState((prevState) => {
      return { ...prevState, frame: 'performance' };
    });
  }

  onBuisnessClicked() {
    this.setState((prevState) => {
      return { ...prevState, frame: 'buisness' };
    });
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

  getFrameColor(frame) {
    switch (frame) {
      case 'safety':
        return SAFETY_COLOR;
      case 'quality':
        return QUALITY_COLOR;
      case 'performance':
        return PERFORMANCE_COLOR;
      case 'buisness':
        return BUISNESS_COLOR;
      default:
        return null;
    }
  }

  generateButtonLayout(frame) {
    let frameButtons = null;
    if (frame === 'safety') {
      frameButtons = (
        <Stack horizontal>
          <PrimaryButton
            text="Safety"
            onClick={this.onSafetyClicked}
            styles={{
              root: {
                backgroundColor: this.getFrameColor('safety'),
                borderColor: this.getFrameColor('safety'),
                borderRadius: '0',
              },
              rootHovered: {
                backgroundColor: this.getFrameColor('safety'),
                borderColor: this.getFrameColor('safety'),
              },
              rootPressed: {
                backgroundColor: this.getFrameColor('safety'),
                borderColor: this.getFrameColor('safety'),
              },
            }}
          />
          <DefaultButton
            text="Program Quality"
            onClick={this.onQualityClicked}
            styles={{
              root: { color: this.getFrameColor('quality'), borderRadius: '0' },
            }}
          />
          <DefaultButton
            text="Robot Performance"
            onClick={this.onPerformanceClicked}
            styles={{
              root: {
                color: this.getFrameColor('performance'),
                borderRadius: '0',
              },
            }}
          />
          <DefaultButton
            text="Buisness Objectives"
            onClick={this.onBuisnessClicked}
            styles={{
              root: {
                color: this.getFrameColor('buisness'),
                borderRadius: '0',
              },
            }}
          />
        </Stack>
      );
    } else if (frame === 'quality') {
      frameButtons = (
        <Stack horizontal>
          <DefaultButton
            text="Safety"
            onClick={this.onSafetyClicked}
            styles={{
              root: { color: this.getFrameColor('safety'), borderRadius: '0' },
            }}
          />
          <PrimaryButton
            text="Program Quality"
            onClick={this.onQualityClicked}
            styles={{
              root: {
                backgroundColor: this.getFrameColor('quality'),
                borderColor: this.getFrameColor('quality'),
                borderRadius: '0',
              },
              rootHovered: {
                backgroundColor: this.getFrameColor('quality'),
                borderColor: this.getFrameColor('quality'),
              },
              rootPressed: {
                backgroundColor: this.getFrameColor('quality'),
                borderColor: this.getFrameColor('quality'),
              },
            }}
          />
          <DefaultButton
            text="Robot Performance"
            onClick={this.onPerformanceClicked}
            styles={{
              root: {
                color: this.getFrameColor('performance'),
                borderRadius: '0',
              },
            }}
          />
          <DefaultButton
            text="Buisness Objectives"
            onClick={this.onBuisnessClicked}
            styles={{
              root: {
                color: this.getFrameColor('buisness'),
                borderRadius: '0',
              },
            }}
          />
        </Stack>
      );
    } else if (frame === 'performance') {
      frameButtons = (
        <Stack horizontal>
          <DefaultButton
            text="Safety"
            onClick={this.onSafetyClicked}
            styles={{
              root: { color: this.getFrameColor('safety'), borderRadius: '0' },
            }}
          />
          <DefaultButton
            text="Program Quality"
            onClick={this.onQualityClicked}
            styles={{
              root: { color: this.getFrameColor('quality'), borderRadius: '0' },
            }}
          />
          <PrimaryButton
            text="Robot Performance"
            onClick={this.onPerformanceClicked}
            styles={{
              root: {
                backgroundColor: this.getFrameColor('performance'),
                borderColor: this.getFrameColor('performance'),
                borderRadius: '0',
              },
              rootHovered: {
                backgroundColor: this.getFrameColor('performance'),
                borderColor: this.getFrameColor('performance'),
              },
              rootPressed: {
                backgroundColor: this.getFrameColor('performance'),
                borderColor: this.getFrameColor('performance'),
              },
            }}
          />
          <DefaultButton
            text="Buisness Objectives"
            onClick={this.onBuisnessClicked}
            styles={{
              root: {
                color: this.getFrameColor('buisness'),
                borderRadius: '0',
              },
            }}
          />
        </Stack>
      );
    } else if (frame === 'buisness') {
      frameButtons = (
        <Stack horizontal>
          <DefaultButton
            text="Safety"
            onClick={this.onSafetyClicked}
            styles={{
              root: { color: this.getFrameColor('safety'), borderRadius: '0' },
            }}
          />
          <DefaultButton
            text="Program Quality"
            onClick={this.onQualityClicked}
            styles={{
              root: { color: this.getFrameColor('quality'), borderRadius: '0' },
            }}
          />
          <DefaultButton
            text="Robot Performance"
            onClick={this.onPerformanceClicked}
            styles={{
              root: {
                color: this.getFrameColor('performance'),
                borderRadius: '0',
              },
            }}
          />
          <PrimaryButton
            text="Buisness Objectives"
            onClick={this.onBuisnessClicked}
            styles={{
              root: {
                backgroundColor: this.getFrameColor('buisness'),
                borderColor: this.getFrameColor('buisness'),
                borderRadius: '0',
              },
              rootHovered: {
                backgroundColor: this.getFrameColor('buisness'),
                borderColor: this.getFrameColor('buisness'),
              },
              rootPressed: {
                backgroundColor: this.getFrameColor('buisness'),
                borderColor: this.getFrameColor('buisness'),
              },
            }}
          />
        </Stack>
      );
    }

    return frameButtons;
  }

  render() {
    const { frame } = this.state;
    const { theme, width, height } = this.props;

    const padding = 5;

    const outerWidth = width;
    const outerHeight = height;
    const innerWidth = outerWidth - 2 * padding;
    const innerHeight = outerHeight - 2 * padding;

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
        <Stack>
          {this.generateButtonLayout(frame)}

          <div
            style={{
              padding: '5px',
              backgroundColor: this.getFrameColor(frame),
            }}
          >
            <Unity unityContent={this.unityContent} />
          </div>

          <Stack.Item align="center">
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
          </Stack.Item>
        </Stack>
      </div>
    );
  }
}

export default Simulator;
