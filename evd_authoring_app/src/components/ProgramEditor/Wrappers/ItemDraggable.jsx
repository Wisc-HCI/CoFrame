import React, {useCallback, useRef} from 'react';
import { useDrag } from 'react-dnd';
import useEvdStore, {typeToKey} from '../../../stores/EvdStore';
import {childLookup} from './childLookup';

export function ItemDraggable({id, itemType, ancestors}) {

  const ref = useRef(null);

  const data = useEvdStore(useCallback(state=>state.data[typeToKey(itemType)][id],[id,itemType]));

  const Child = childLookup[itemType];

  const [{opacity}, drag] = useDrag(()=>({
    type: data.type,
    item: data,
    collect: monitor => ({
      opacity: monitor.isDragging() ? 0.4 : 1
    })
  }))

  drag(ref)
  
  return data ? <Child ref={ref} style={{opacity}} data={data} ancestors={ancestors}/> : null
}
