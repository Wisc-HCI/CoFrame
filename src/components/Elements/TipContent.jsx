// import { Box, Text } from 'grommet'
import { Tooltip } from "@mui/material"

export const TipText = ({ previewText, extraText, color}) => {
    return (
        <Tooltip
            style={{
                backgroundColor:color,
                color:'black'
            }}
            title={extraText}
            arrow
        >
             {previewText}
            
        </Tooltip>
    )
}