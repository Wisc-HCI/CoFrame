import React from 'react';

import { 
    Stack,
    ScrollablePane 
} from 'office-ui-fabric-react';

import { ThemeContext } from '../../contexts';


export const Set = (props) => {

    const { children } = props;

    return (
        <ThemeContext.Consumer>
            { value => (
                <div
                    style={{
                        position: 'relative',
                        width: '90%', 
                        height: '100%',
                        backgroundColor: value.theme.semanticColors.bodyStandoutBackground
                    }}
                >
                    <ScrollablePane>
                        <Stack
                            styles={{ root: {
                                paddingTop: '10px',
                                paddingLeft: '10px',
                                paddingRight: '20px', 
                            }}}
                        >
                            {children}
                        </Stack>    
                        
                    </ScrollablePane>
                </div>
            )}
        </ ThemeContext.Consumer>
    );
};


