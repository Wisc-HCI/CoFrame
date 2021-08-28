import React, {useCallback, useRef} from 'react';
import { useDrag } from 'react-dnd';
import useStore from '../../../stores/Store';
import { typeToKey } from '../../../stores/helpers';
import {childLookup} from './childLookup';

export function ItemDraggable({id, itemType, ancestors, context, disabled}) {

  const ref = useRef(null);

  const data = useStore(useCallback(state=>state.data[typeToKey(itemType)][id],[id,itemType]));

  const Child = childLookup[itemType];

  const [{opacity}, drag, preview] = useDrag(()=>({
    type: data.type,
    item: data,
    collect: monitor => ({
      opacity: monitor.isDragging() ? 0.4 : 1
    })
  }))

  drag(ref)
  
  return data ? <Child ref={disabled ? null : ref} preview={disabled ? null : preview} style={{opacity}} data={data} ancestors={ancestors} context={context}/> : null
}
