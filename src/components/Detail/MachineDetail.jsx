import React from 'react';
import useStore from '../../stores/Store';
import { Box, Text } from 'grommet';
import { ExternalBlock, referenceTemplateFromSpec } from "simple-vp";
import Collapse from '../Collapse';

export const MachineProcessList = ({ machineId }) => {

  const processTypeInfo = useStore(state => state.programSpec.objectTypes.processType)

  const addFocusItem = useStore(state => state.addFocusItem);

  const processList = useStore(state => Object.values(state.programData)
       .filter(value => value.type === 'processType' && value.properties.machine === machineId));

  return (
    <Collapse
      openable={true}
      borderWidth={3}
      defaultOpen={true}
      style={{backgroundColor:'#303030',marginBottom: 5}}
      backgroundColor='#202020'
      header={<Box direction='row' pad="10pt">Process: </Box>}
    >

      {processList.length > 0 ? processList.map(process => {
        const processRef = referenceTemplateFromSpec('processType', process, processTypeInfo);
        return (
          <Box key={process.id} round="xsmall" background="rgba(100,100,100,0.3)" direction='column'
            elevation="none" pad="xsmall" justify='center'
            hoverIndicator={true}
            onClick={() => {
              addFocusItem(process.id, true);
            }}>
            <ExternalBlock
              store={useStore}
              data={processRef}
              highlightColor={"#333333"}
            />
          </Box>
        )
      }) : (
        <Text alignSelf='center'>No Associated Processes</Text>
      )}
    </Collapse>
  )
}

