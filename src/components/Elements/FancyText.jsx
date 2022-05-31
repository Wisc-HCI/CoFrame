import {styled, keyframes} from '@stitches/react';

const appear = keyframes({
    '0%': { transform: 'scale(0)', opacity: 0 },
    '45%': { transform: 'scale(0.7)' },
    '65%': { transform: 'scale(1.1)', opacity: 1 },
    '75%': { transform: 'scale(1.03)' },
    '100%': { transform: 'scale(1)' },
  });

export const FancyText = styled('p', {
    animation: `${appear} 500ms`,
  });