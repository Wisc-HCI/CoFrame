import React from 'react';
import useStore from '../../stores/Store';
import { Box, Text } from 'grommet';
import { ExternalBlock, referenceTemplateFromSpec, instanceTemplateFromSpec } from "simple-vp";


export const FixtureItem = ({ fixtureID }) => {

    const fixtureTypeInfo = useStore(state => state.programSpec.objectTypes.fixtureType);
    


    const addFocusItem = useStore(state => state.addFocusItem);

    const fixtureItem = useStore(state => state.programData[fixtureID]);

    

    const fixtureRef = referenceTemplateFromSpec('fixtureType', fixtureItem, fixtureTypeInfo);

    



    return (
        <>
        
        <Box round="xsmall" pad="small" background="#303030" wrap = {true}>
            <b style={{ color: 'rgba(255, 255, 255, 0.85)', paddingBottom: '2%' }} >Relative to : </b>
            <div>
            <Box round="xsmall" background="rgba(100,100,100,0.3)" direction='column'
                elevation="none" pad="xsmall" justify='center'
                hoverIndicator={true}
                onClick={() => {
                    addFocusItem(fixtureID, true);
                }}>
                <Box>
                <ExternalBlock
                    store={useStore}
                    data={fixtureRef}
                    highlightColor={"#333333"}
                />
                </Box>
            </Box>        
            </div>
          </Box>
            
        </>






    )
}

