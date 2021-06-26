import React from 'react';
import {useDraggable} from '@dnd-kit/core';
import {CSS} from '@dnd-kit/utilities';
import useEvdStore from '../../../stores/EvdStore';
import useGuiStore from '../../../stores/GuiStore';
import {childLookup} from './childLookup';
import { acceptLookup } from '../acceptLookup';

export function ProgramDraggable(_) {

  const data = useEvdStore(state=>({
    name:state.name,
    type:state.type,
    uuid:state.uuid,
    primitiveIds:state.primitiveIds,
    description:state.description,
    transform:state.transform
  }))

  const dragItem = useGuiStore(state=>state.dragItem)

  const ancestors = [
    {uuid:'grid',...acceptLookup.grid.primitiveIds}
  ];

  const {
    attributes,
    listeners,
    setNodeRef,
    transition,
  } = useDraggable({id: data.uuid, data:{uuid:data.uuid,idx:0,ancestors,itemType:'program',action:'itemDrag',record:data}});

  const Child = childLookup['program'];

  // console.log(transform)
  
  const style = {
    opacity: dragItem && dragItem.uuid === data.uuid ? 0 : 1,
    transform: CSS.Transform.toString({scaleX:1,scaleY:1,...data.transform}),
    transition,
  };
  
  return <Child ref={setNodeRef} style={style} {...attributes} {...listeners} data={data} ancestors={ancestors}/>
}
