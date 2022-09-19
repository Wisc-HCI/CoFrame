import { Button } from '@mui/material';
import React from 'react';

export const FrameButton = ({ frame, active, onClick, text }) => {
    return (
        <Button
            variant="outlined"
            size='small'
            color={active ? 'black' : frame}
            onClick={onClick}
            style={{padding: 5}}
        >
            {text}
        </Button>
    )
}