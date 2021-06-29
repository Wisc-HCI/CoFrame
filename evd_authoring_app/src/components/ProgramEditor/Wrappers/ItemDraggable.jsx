import React, {useCallback} from 'react';
import { useDrag } from 'react-dnd';
import useEvdStore, {typeToKey} from '../../../stores/EvdStore';
import {childLookup} from './childLookup';

export function ItemDraggable({id, itemType, ancestors}) {

  const data = useEvdStore(useCallback(state=>state.data[typeToKey(itemType)][id],[id,itemType]));

  const Child = childLookup[itemType];

  const [{opacity}, drag] = useDrag({
    item: data,
    collect: monitor => ({
      opacity: monitor.isDragging() ? 0.4 : 1
    })
  })
  
  return <Child ref={drag} style={{opacity}} data={data} ancestors={ancestors}/>
}
