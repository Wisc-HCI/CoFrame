import React from 'react';
import {useSortable} from '@dnd-kit/sortable';
import {CSS} from '@dnd-kit/utilities';
import {childLookup} from './childLookup';

export function GenericSortable({idx, itemType, data, ancestors, hide}) {

  const Child = childLookup[itemType];

  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
  } = useSortable({id: data.uuid, data:{uuid:data.uuid,idx,ancestors,itemType,action:'genericSort',record:data}});
  
  const style = {
    opacity: hide ? 0 : 1,
    transform: CSS.Transform.toString(transform),
    transition,
  };
  
  return <Child ref={setNodeRef} style={style} {...attributes} {...listeners} data={data} idx={idx}/>
}
