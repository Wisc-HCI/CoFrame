import React from 'react';

// import { DndProvider } from 'react-dnd';
import {
  DndContext, 
  closestCenter,
  KeyboardSensor,
  PointerSensor,
  useSensor,
  useSensors,
} from '@dnd-kit/core';
import {
  sortableKeyboardCoordinates
} from '@dnd-kit/sortable';
// import { HTML5Backend } from 'react-dnd-html5-backend';
// import { MultiBackend } from 'react-dnd-multi-backend'
import { Editor } from './Editor';

export const ProgramEditor = (_) => {

  const sensors = useSensors(
    useSensor(PointerSensor),
    useSensor(KeyboardSensor, {
      coordinateGetter: sortableKeyboardCoordinates,
    })
  );

  function handleDragEnd(event) {
    const {active, over} = event;
    if (active.id !== over.id) {
      console.log({active, over})
    }
  }

  return (
    <div style={{ width: '100%', height: 'calc(100vh - 113pt)', display: 'flex',borderRadius:3 }}>
      <DndContext 
        sensors={sensors}
        collisionDetection={closestCenter}
        onDragEnd={handleDragEnd}
      >
        <Editor/>
      </DndContext>
      
    </div>
  )

}
