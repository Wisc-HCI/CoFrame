import React from 'react';
import { DndProvider } from 'react-dnd'
import { HTML5Backend } from 'react-dnd-html5-backend'
import {ProgramCanvas} from './ProgramCanvas';

// We might want to look into the ResizeObserver for updating
// the size of this component more gracefully. Right now it
// initializes too large, and if the screen is resized it breaks.
// import { ResizeObserver } from "@juggle/resize-observer";

export const ProgramEditor = (_) => {
  
  return (
    <div style={{width:'100%',height:'100%'}}>
      <DndProvider backend={HTML5Backend}>
        <ProgramCanvas/>
      </DndProvider>
    </div>
  )

}