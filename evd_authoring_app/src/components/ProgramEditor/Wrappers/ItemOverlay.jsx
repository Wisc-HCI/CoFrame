import React, {useCallback} from 'react';
import useEvdStore, {typeToKey} from '../../../stores/EvdStore';
import {childLookup} from './childLookup';

export function ItemOverlay({id, itemType}) {
  
  const data = useEvdStore(useCallback(state=>state.data[typeToKey(itemType)][id],[id,itemType]));
  console.log(data)

  const Child = childLookup[itemType];
  
  return (
    <Child data={data} style={{boxShadow:'2pt 2pt 8pt rgba(0,0,0,0.4)'}}/>
  );
}
