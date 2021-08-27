import React, {useCallback, useRef} from 'react';
import { useDrag, useDrop } from 'react-dnd';
import useStore from '../../../stores/Store';
import { typeToKey } from '../../../stores/helpers';
import {childLookup} from './childLookup';

const validDrop=(item,ancestors) => ancestors[0].accepts.indexOf(item.type)>=0 && (ancestors.map(ancestor=>ancestor.uuid).indexOf(item.parentData.uuid)>=0 || item.parentData.type === 'drawer');

export function ItemSortable({id, itemType, ancestors, context, disabled, onMove}) {

  const ref = useRef(null);

  const data = useStore(useCallback(state=>state.data[typeToKey(itemType)][id],[id,itemType]));

  const Child = childLookup[itemType];

  const [{opacity}, drag, preview] = useDrag(()=>({
    type: data.type,
    item: data,
    collect: monitor => ({
      opacity: monitor.isDragging() ? 0.4 : 1
    })
  }))

  // We only care about the second value returned from useDrop (hence the [1] at the end)
  const drop = useDrop({
    accept: ancestors[0].accepts,
    drop: (item, _) => {
      onMove(item)
    },
    hover: (item, _) => {
      if (!ref.current || item.uuid === id) {
        return
      }
      if (validDrop(item,ancestors)) {
        onMove(item)
      }
    },
    canDrop: (item, _) => validDrop(item,ancestors)
  })[1]

  drag(drop(ref))

  // In case there is some lag in updating the store, only render the component if there is actually data.

  return data ? <Child key={data.uuid} ref={disabled ? null : ref} preview={disabled ? null : preview} style={{opacity}} data={data} ancestors={ancestors} context={context}/> : null
}