import React, {useRef} from 'react';
import { useDrag } from 'react-dnd';
import {childLookup} from './childLookup';

export function GenericDraggable({itemType, data, ancestors}) {

  const ref = useRef(null);

  const Child = childLookup[itemType];

  const [{opacity}, drag] = useDrag({
    type: data.type,
    item: data,
    collect: monitor => ({
      opacity: monitor.isDragging() ? 0.4 : 1
    })
  })

  drag(ref);

  return <Child ref={ref} style={{opacity}} data={data} ancestors={ancestors}/>
}
