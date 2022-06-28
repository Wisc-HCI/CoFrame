import React from 'react';
import useStore from '../../stores/Store';
import { Box, Text } from 'grommet';
import { ExternalBlock, referenceTemplateFromSpec } from "simple-vp";
import Collapse from '../Elements/Collapse';

export const GizmoDetail = ({ gizmoId }) => {

  
  

  const addFocusItem = useStore(state => state.addFocusItem);
  
  const gizmoList = useStore(state => {
    return Object.values(state.programData).filter(value =>  value.id === gizmoId
    )

  });
  const gizmoMachineInfo = useStore(state => state.programSpec.objectTypes.machineType);
  const gizmoToolInfo = useStore(state => state.programSpec.objectTypes.toolType);



 

  

  return (
    <Collapse
      openable={true}
      borderWidth={3}
      defaultOpen={true}
      style={{backgroundColor:'#303030',marginBottom: 5}}
      backgroundColor='#202020'
      header={<Box direction='row' pad="10pt"> 
      <b style={{ color: 'rgba(255, 255, 255, 0.85)' }} >Gizmo : </b>
      </Box>}
    >

      {gizmoList ? gizmoList.map(gizmo => {
        const gizmoRef = gizmo.type === 'machineType' ? referenceTemplateFromSpec('machineType', gizmo, gizmoMachineInfo) : referenceTemplateFromSpec('toolType', gizmo, gizmoToolInfo) ;
        return (
          <Box key={gizmo.id} round="xsmall" background="rgba(100,100,100,0.3)" direction='column'
            elevation="none" pad="xsmall" justify='center'
            hoverIndicator={true}
            onClick={() => {
              addFocusItem(gizmo.id, true);
            }}>
            <ExternalBlock
              store={useStore}
              data={gizmoRef}
              highlightColor={"#333333"}
            />
          </Box>
        )
      }) : (
        <Text alignSelf='center'>No Associated Gizmo</Text>
      )}
    </Collapse>
  )
}

