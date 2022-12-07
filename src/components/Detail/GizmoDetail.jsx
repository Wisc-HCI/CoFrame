import React from 'react';
import useStore from '../../stores/Store';
import { ExternalBlock, referenceTemplateFromSpec } from "simple-vp";
import {Collapse} from '../Elements/Collapse';
import { Stack, Typography } from '@mui/material';

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
      header='Gizmo'
      defaultOpen
    >

      {gizmoList ? gizmoList.map(gizmo => {
        const gizmoRef = gizmo.type === 'machineType' ? referenceTemplateFromSpec('machineType', gizmo, gizmoMachineInfo) : referenceTemplateFromSpec('toolType', gizmo, gizmoToolInfo) ;
        return (
          <Stack key={gizmo.id} style={{borderRadius:2,backgroundColor:"rgba(100,100,100,0.3)",padding:1,justify:'center'}} direction='column'
            onClick={() => {
              addFocusItem(gizmo.id, true);
            }}>
            <ExternalBlock
              store={useStore}
              data={gizmoRef}
              highlightColor={"#333333"}
            />
          </Stack>
        )
      }) : (
        <Typography style={{alignSelf:"center"}}>No Associated Items</Typography>
      )}
    </Collapse>
  )
}

