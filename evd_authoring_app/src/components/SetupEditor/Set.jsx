import React from 'react';

import { Stack } from '@fluentui/react/lib/Stack';
import { ScrollablePane } from '@fluentui/react/lib/ScrollablePane';
// Sticky not available? Was used in Stack.Item but might have to look at what to swap that out with

import { AddButton } from './AddButton';
import { ThemeContext } from '../../contexts';


export const Set = (props) => {

    const { sets } = props;

    const sections = sets.map(s => {
        return (
            <Stack.Item 
                key={s.name}
                align="stretch"
                styles={{
                    root: {
                        paddingBottom: '20px'
                    }
                }}
            >
                <>
                    <div style={{fontSize: '25px', paddingBottom: '5px'}}>{s.name}</div>
                    <AddButton type={s.type} callback={() => {}} disabled={ s.addable === undefined ? false : !s.addable} />
                    <br />
                </>

                {s.content}

            </Stack.Item>
        );
    });

    return (
        <ThemeContext.Consumer>
            { value => (
                <div
                    style={{
                        paddingTop: '10px',
                        paddingLeft: '10px',
                        paddingRight: '20px', 
                        width: '95%', 
                        height: '100%',
                        backgroundColor: value.theme.semanticColors.bodyStandoutBackground
                    }}
                >
                    <div
                        style={{
                            width: '100%', 
                            height: '100%',
                            position: 'relative',
                        }}
                    >
                        <ScrollablePane>
                            <Stack>
                                {sections}
                            </Stack>
                        </ScrollablePane>
                    </div>
                </div>
            )}
        </ ThemeContext.Consumer>
    );
};