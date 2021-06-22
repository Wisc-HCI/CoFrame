import React from 'react';
import {fromTemplate} from '../../../stores/templates';
import {childLookup} from './childLookup';

export function GenericOverlay({type, itemType}) {

  const data = fromTemplate(type);

  const Child = childLookup[itemType];
  
  return (
    <Child data={data} style={{boxShadow:'2pt 2pt 8pt rgba(0,0,0,0.4)'}}/>
  );
}
