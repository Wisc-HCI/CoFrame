import React from 'react';
import {useDraggable} from '@dnd-kit/core';
import {CSS} from '@dnd-kit/utilities';
import {childLookup} from './childLookup';

export function GenericDraggable({itemType, data, ancestors, hide}) {

  const Child = childLookup[itemType];

  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
  } = useDraggable({id: data.uuid, data:{uuid:data.uuid,ancestors,itemType,action:'genericSort',record:data}});
  
  const style = {
    opacity: hide ? 0 : 1,
    transform: CSS.Transform.toString(transform),
    transition,
  };
  
  return <Child ref={setNodeRef} style={style} {...attributes} {...listeners} data={data} idx={0}/>
}
