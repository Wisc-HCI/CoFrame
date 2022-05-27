import {styled, keyframes} from '@stitches/react';

const appear = keyframes({
    '0%': { transform: 'scale(0)', opacity: 0 },
    '45%': { transform: 'scale(0.7)' },
    '65%': { transform: 'scale(1.1)', opacity: 1 },
    '75%': { transform: 'scale(1.03)' },
    '100%': { transform: 'scale(1)' },
  });

  0, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 1

  1, 0.97, 0.9, 1.1, 0.9, 1.1, 1.03, 1

export const FancyText = styled('p', {
    animation: `${appear} 500ms`,
  });