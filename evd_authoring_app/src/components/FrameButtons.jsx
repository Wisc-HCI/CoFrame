import React from 'react';

import styles from '../frameStyles';

import {
  Stack,
  DefaultButton,
  PrimaryButton,
} from 'office-ui-fabric-react';


export const StyledButton = (props) => {

  const { primary, callback, frame, text } = props;

  const frameColor = styles.colors[frame];

  let button = null;
  if (primary) {
    button = (
      <PrimaryButton
        text={text}
        onClick={callback}
        styles={{
          root: {
            backgroundColor: frameColor,
            borderColor: frameColor,
            borderRadius: '0',
          },
          rootHovered: {
            backgroundColor: frameColor,
            borderColor: frameColor,
          },
          rootPressed: {
            backgroundColor: frameColor,
            borderColor: frameColor,
          },
        }}
      />
    );
  } else {
    button = (
      <DefaultButton
        text={text}
        onClick={callback}
        styles={{
          root: {
            color: frameColor,
            borderRadius: '0',
          },
        }}
      />
    );
  }

  return (
    <React.Fragment>
      {button}
    </React.Fragment>
  );
};


export const FrameButtons = (props) => {

    const { frame, callback } = props;

    return (
      <Stack horizontal>
        
        <StyledButton 
          text="Safety" 
          frame="safety"
          primary={frame === 'safety'} 
          callback={() => callback('safety')}
        />

        <StyledButton 
          text="Program Quality" 
          frame="quality"
          primary={frame === 'quality'} 
          callback={() => callback('quality')}
        />
        
        <StyledButton 
          text="Robot Performance" 
          frame="performance"
          primary={frame === 'performance'} 
          callback={() => callback('performance')}
        />
        
        <StyledButton
          text="Buisness Objectives" 
          frame="buisness"
          primary={frame === 'buisness'} 
          callback={() => callback('buisness')}
        />

      </Stack>
    );
};