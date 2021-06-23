import React from 'react';
import {useDraggable} from '@dnd-kit/core';
import {CSS} from '@dnd-kit/utilities';
import {fromTemplate} from '../../../stores/templates';
import {childLookup} from './childLookup';

export function GenericDraggable({type, itemType, source, hide}) {

  const data = fromTemplate(type);
  const Child = childLookup[itemType];

  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
  } = useDraggable({id: type, data:{uuid:data.uuid,source,type,itemType,action:'genericDrag',initial:data}});

  const style = {
    opacity: hide ? 0 : 1,
    transform: CSS.Transform.toString(transform),
    transition,
  };
  
  return (
    <Child ref={setNodeRef} style={style} {...attributes} {...listeners} data={data}/>
  );
}
