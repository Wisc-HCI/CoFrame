import { Box, Text } from 'grommet'

export const TipContent = ({ message, color, width, inverted }) => (
    <Box direction="column" align="center">
        {inverted && (
            <svg viewBox="0 0 22 22" version="1.1" width="22px" height="22px">
                <polygon
                    fill={color ? color : 'grey'}
                    points="0 22 11 12 22 22"
                />
            </svg>
        )}
        <Box background={color ? color : 'grey'} direction="row" pad="small" round="xxsmall" width={width}>
            <Text color="#313131" >{message}</Text>
        </Box>
        {!inverted && (
            <svg viewBox="0 0 22 22" version="1.1" width="22px" height="22px">
                <polygon
                    fill={color ? color : 'grey'}
                    points="22 0 11 12 0 0"
                />
            </svg>
        )}

    </Box>
);

export const TipText = ({ previewText, extraText, color}) => {
    return (
        <Text
            color={color}
            tip={{
                content: <TipContent message={extraText} color={color} width='200pt' />,
                plain: true,
                dropProps: {
                    align: { bottom: 'top' }
                }
            }}
        >
             {previewText}
            
        </Text>
    )
}