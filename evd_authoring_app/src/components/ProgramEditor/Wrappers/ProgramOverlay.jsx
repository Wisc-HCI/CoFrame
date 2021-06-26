import React from 'react';
import useEvdStore from '../../../stores/EvdStore';
import {childLookup} from './childLookup';
import {CSS} from '@dnd-kit/utilities';
import { acceptLookup } from '../acceptLookup';

export function ProgramOverlay(_) {

  const data = useEvdStore(state=>({
    name:state.name,
    type:state.type,
    uuid:state.uuid,
    primitiveIds:state.primitiveIds,
    description:state.description,
    transform:state.transform
  }))

  const ancestors = [
    {uuid:'grid',...acceptLookup.grid.primitiveIds}
  ];

  const cssTransform = CSS.Transform.toString({scaleX:1,scaleY:1,...data.transform})
  const Child = childLookup['program'];
  
  return <Child data={data} ancestors={ancestors} style={{boxShadow:'2pt 2pt 8pt rgba(0,0,0,0.4)', transform:cssTransform}}/>
}
