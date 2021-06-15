import React from 'react';
import { useDrop } from 'react-dnd'
import {ProgramBlock} from './ProgramBlock';
import useEvdStore from '../../stores/EvdStore';


export const ProgramCanvas = (_) => {
  
    const setTransform = useEvdStore(state=>state.setTransform)
    const [, drop] = useDrop(() => ({
      accept: 'program',
      drop(item, monitor) {
          console.log('DROPPING!')
          const delta = monitor.getDifferenceFromInitialOffset();
          const left = Math.round(item.left + delta.x);
          const top = Math.round(item.top + delta.y);
          setTransform({left, top});
          return undefined;
      },
    }), [setTransform]);
    
    return (
      <div ref={drop} style={{width:'100%',height:'100%',position:'relative',overflow:'auto'}}>
        <ProgramBlock/>
      </div>
    )
  
  }