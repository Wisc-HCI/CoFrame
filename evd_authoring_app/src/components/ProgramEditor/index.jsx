import React from 'react';

import { DndProvider } from 'react-dnd';
import { HTML5Backend } from 'react-dnd-html5-backend';
import { Editor } from './Editor';

export const ProgramEditor = (_) => {

  return (
    <div style={{ width: '100%', height: '100%', display: 'flex' }}>
      <DndProvider backend={HTML5Backend}>
        <Editor/>
      </DndProvider>
    </div>
  )

}
