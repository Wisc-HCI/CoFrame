import { React, useState } from 'react';
import { LocationDetail } from './LocationDetail';
import { MachineDetail } from './MachineDetail';
import { ThingDetail } from './ThingDetail';
import { WaypointDetail } from './WaypointDetail';
import { ProcessDetail } from './ProcessDetail';
import { InputOutputDetail } from './InputOutputDetail';

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

  const [processItem, setProcessItem] = useState(null);
  const [inputOutputItem, setInputOutputItem] = useState(null);
  const [position, setPosition] = useState(null);
  const [orientation, setOrientation] = useState(null);

  const data = useStore(state => state.programData);



  function handleProcessClick(id) {
    //console.log(id);
    for (const [key, value] of Object.entries(data)) {
      if (value.type === "processType" && value.id === id) {
        //console.log('entered1');
        setProcessItem(value);
      }
    }
  }

  function handleInputOutputClick(id, position, orientation) {
    console.log(id);
    console.log(position);
    console.log(orientation);
    for (const [key, value] of Object.entries(data)) {
      if (value.id === id) {
        setInputOutputItem(value);
        setPosition(position);
        setOrientation(orientation);
      }
    }

  }

  if (inputOutputItem !== null) {
    console.log("InputOutputDetial", item);
    return (
      <>
        <InputOutputDetail item={inputOutputItem} position={position} orientation={orientation} />
      </>
    )

  }
  else if (processItem !== null) {

    return (
      <>
        {processItem.type === 'processType' && (
          <><ProcessDetail item={processItem}
            inputOutputClick={(id, position, orientation) => handleInputOutputClick(id, position, orientation)} /></>
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
              <MachineDetail item={item} objectTypeInfo={objectTypeInfo} processClick={(id) => handleProcessClick(id)} />
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

