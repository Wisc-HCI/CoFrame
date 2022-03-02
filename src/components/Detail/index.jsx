import React from 'react';
import { LocationDetail } from './LocationDetail';
import { MachineDetail } from './MachineDetail';
import { ThingDetail } from './ThingDetail';
import { WaypointDetail } from './WaypointDetail';

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

  //console.log("item is ", focusItem);

  if (focusItem.uuid && !EDITOR_TYPES.includes(focusItem.type)) {
    return (
      <>
        {item.type === 'locationType' && (
          <LocationDetail uuid={item.id} />
        )}
        {item.type === 'machineType' && (
          <>
            <div>
              <MachineDetail item={item} objectTypeInfo={objectTypeInfo} />
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

