import React, {useCallback, useRef} from 'react';
import { useDrag, useDrop, useDragDropManager } from 'react-dnd';
import useEvdStore, {typeToKey} from '../../../stores/EvdStore';
import useGuiStore from '../../../stores/GuiStore';
import {childLookup} from './childLookup';

export function ItemSortable({id, idx, itemType, ancestors}) {

  const ref = useRef(null);

  const data = useEvdStore(useCallback(state=>state.data[typeToKey(itemType)][id],[id,itemType]));

  const moveChildPrimitive = useEvdStore(state=>state.moveChildPrimitive);

  // const [dragItem, setDragItem, clearDragItem] = useGuiStore(state=>([state.dragItem,state.setDragItem,state.clearDragItem]));

  const Child = childLookup[itemType];

  const [{opacity}, drag] = useDrag(()=>({
    type: data.type,
    item: data,
    collect: monitor => ({
      opacity: monitor.isDragging() ? 0.4 : 1
    })
  }))

  const [{}, drop] = useDrop({
    accept: ancestors[0].accepts,
    drop: (item, _) => {
      console.log('drop')
    },
    hover: (item, _) => {
      if (item.editable) {
        moveChildPrimitive(item,ancestors[0].uuid,idx)
      }
    },
    canDrop: (item, _) => (ancestors[0].accepts.indexOf(item.type)>=0 && item.editable),

  })

  drag(drop(ref))

  // In case there is some lag in updating the store, only render the component if there is actually data.

  return data ? <Child ref={ref} style={{opacity}} data={data} ancestors={ancestors} idx={idx}/> : null
}