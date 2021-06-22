import React from 'react';
import {useSortable} from '@dnd-kit/sortable';
import {CSS} from '@dnd-kit/utilities';
import {fromTemplate} from '../../../stores/templates';
import {childLookup} from './childLookup';

export function GenericSortable({type, itemType, source, hide}) {

  const data = fromTemplate(type);
  const Child = childLookup[itemType];

  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
  } = useSortable({id: type, data:{source,type,itemType,action:'genericSort',initial:data}});
  
  const style = {
    opacity: hide ? 0 : 1,
    transform: CSS.Transform.toString(transform),
    transition,
  };
  
  return (
    <Child ref={setNodeRef} style={style} {...attributes} {...listeners} data={data}/>
  );
}
