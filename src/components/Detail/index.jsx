import {React,useState} from 'react';
import { LocationDetail } from './LocationDetail';
import { MachineDetail } from './MachineDetail';
import { ThingDetail } from './ThingDetail';
import { WaypointDetail } from './WaypointDetail';
import { ProcessDetail } from './ProcessDetail';

import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';

const EDITOR_TYPES = ['primitive', 'skill', 'program', 'trajectory', 'scene']



export const Detail = (_) => {

  const {
    focusItem,
    item,
    objectTypeInfo
  } = useStore(state => ({
    focusItem: state.focusItem,
    item: state.focusItem.type ? state.programData[state.focusItem.uuid] : null,
    objectTypeInfo: state.focusItem.type ? state.programSpec.objectTypes[state.programData[state.focusItem.uuid].type] : null
  }), shallow);

  const [processItem,setProcessItem] = useState(null);

  const data = useStore(state => state.programData);

  

  function handleProcessClick(id){
    //console.log(id);
    for (const [key, value] of Object.entries(data)) {
      if (value.type === "processType" && value.id === id) {
        //console.log('entered1');
        setProcessItem(value);
      }
    }
  }

  console.log(processItem);
  if (processItem !== null){
    console.log('entered2');
    return (
      <>
        {processItem.type === 'processType' && (
          <><ProcessDetail item = {processItem}/></>   
        )}
      </>
    )
  } else if (focusItem.uuid && !EDITOR_TYPES.includes(focusItem.type)) {
    return (
      <>
        {item.type === 'locationType' && (
          <LocationDetail uuid={item.id} />
        )}
        {item.type === 'machineType' && (
          <>
            <div>
              <MachineDetail item={item} objectTypeInfo={objectTypeInfo} processClick = {(id)=>handleProcessClick(id)} />
            </div>
          </>

        )}

        


        {item.type === 'waypointType' && (
          <WaypointDetail uuid={item.id} />
        )}
        {item.type === 'thingTypeType' && (
          <ThingDetail uuid={item.id} />
        )}


      </>
    );
  } else {
    return null
  }


}

