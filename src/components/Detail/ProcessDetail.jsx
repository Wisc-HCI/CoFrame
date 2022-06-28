import React from 'react';
import useStore from '../../stores/Store';
import {Text, Box} from 'grommet';
import {ExternalBlock, referenceTemplateFromSpec } from "simple-vp";
import Collapse from '../Elements/Collapse';

export const ProcessIOList = ({ processId, isInput }) => {

    const ioTypeInfo = useStore(state => state.programSpec.objectTypes.inputOutputType)
    const primaryColor = useStore(state=>state.primaryColor);
  
    const addFocusItem = useStore(state => state.addFocusItem);
  
    const ioList = useStore(state => {
       return isInput 
            ? state.programData[processId].properties.inputs.map(io=>state.programData[io])
            : state.programData[processId].properties.outputs.map(io=>state.programData[io]);
    });
  
    return (
      
      <Collapse
        openable={true}
        borderWidth={3}
        defaultOpen={true}
        style={{backgroundColor:'#303030',marginBottom: 5}}
        backgroundColor='#202020'
        header={<Box direction='row' pad="10pt">{isInput?
          <b style={{ color: 'rgba(255, 255, 255, 0.85)' }} >Inputs : </b> :
          <b style={{ color: 'rgba(255, 255, 255, 0.85)' }} >Outputs : </b>}{' '}
          </Box>}
      >
  
        {ioList.length > 0 ? ioList.map((io,i) => {
          const ioRef = referenceTemplateFromSpec('inputOutputType', io, ioTypeInfo);
          return (
            <Box key={i} round="xsmall" background="rgba(100,100,100,0.3)" direction='column'
              elevation="none" pad="xsmall" justify='center'
              hoverIndicator={true}
              onClick={() => {
                addFocusItem(io.id, true);
              }}>
              <ExternalBlock
                store={useStore}
                data={ioRef}
                highlightColor={primaryColor}
              />
            </Box>
          )
        }) : (
            <Text alignSelf='center'>{isInput ? 'No Inputs' : 'No Outputs'}</Text>
        )}
      </Collapse>
      
    )
  }

