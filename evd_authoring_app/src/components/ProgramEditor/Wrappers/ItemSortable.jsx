import React, {useCallback} from 'react';
import {useSortable} from '@dnd-kit/sortable';
import {CSS} from '@dnd-kit/utilities';
import useEvdStore, {typeToKey} from '../../../stores/EvdStore';
import {childLookup} from './childLookup';

export function ItemSortable({id, itemType, source, hide}) {

  const data = useEvdStore(useCallback(state=>state.data[typeToKey(itemType)][id],[id,itemType]));
  console.log(data)

  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
  } = useSortable({id: id, data:{uuid:id,source,itemType,action:'itemSort'}});

  const Child = childLookup[itemType];
  
  const style = {
    opacity: hide ? 0 : 1,
    transform: CSS.Transform.toString(transform),
    transition,
  };
  
  if (data) {
    return <Child ref={setNodeRef} style={style} {...attributes} {...listeners} data={data}/>
  } else {
    return null;
  }
}
