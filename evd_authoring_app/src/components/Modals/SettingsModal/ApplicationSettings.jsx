import React from 'react';

import { 
    Stack, 
    TextField,
    Label
} from 'office-ui-fabric-react';


export const ApplicationSettings = (props) => {

    const { textCallback } = props;

    return (
        <React.Fragment>
            <Stack horizontal tokens={{ childrenGap: '20' }}>

                <Label>Project Name:</Label>

                <TextField 
                    onChange={textCallback}
                    defaultValue={'Untitled'}
                />

            </Stack>
        </React.Fragment>
    );
};
