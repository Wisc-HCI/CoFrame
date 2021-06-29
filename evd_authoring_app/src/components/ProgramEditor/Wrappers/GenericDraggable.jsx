import React from 'react';
// import {useDraggable} from '@dnd-kit/core';
// import {CSS} from '@dnd-kit/utilities';
import { useDrag } from 'react-dnd';
import {childLookup} from './childLookup';

export function GenericDraggable({itemType, data, ancestors}) {

  const Child = childLookup[itemType];

  const [{opacity}, drag] = useDrag({
    item: {data,ancestors},
    collect: monitor => ({
      opacity: monitor.isDragging() ? 0.4 : 1
    })
  })
  
  return <Child ref={drag} style={{opacity}} data={data} idx={0}/>
}
