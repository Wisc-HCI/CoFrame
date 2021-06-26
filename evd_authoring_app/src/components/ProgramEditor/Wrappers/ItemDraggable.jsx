import React, {useCallback} from 'react';
import {useDraggable} from '@dnd-kit/core';
import {CSS} from '@dnd-kit/utilities';
import useEvdStore, {typeToKey} from '../../../stores/EvdStore';
import {childLookup} from './childLookup';

export function ItemDraggable({id, itemType, ancestors, hide}) {

  const data = useEvdStore(useCallback(state=>state.data[typeToKey(itemType)][id],[id,itemType]));

  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
  } = useDraggable({id: id, data:{uuid:id,idx:0,ancestors,itemType,action:'itemDrag',record:data}});

  const Child = childLookup[itemType];
  
  const style = {
    opacity: hide ? 0 : 1,
    transform: CSS.Transform.toString(transform),
    transition,
  };
  
  return <Child ref={setNodeRef} style={style} {...attributes} {...listeners} data={data} ancestors={ancestors}/>
}
