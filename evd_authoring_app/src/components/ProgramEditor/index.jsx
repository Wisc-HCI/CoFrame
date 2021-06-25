import React, { useState } from 'react';
import {
  DndContext,
  DragOverlay,
  closestCenter,
  KeyboardSensor,
  PointerSensor,
  useSensor,
  useSensors,
} from '@dnd-kit/core';
import {
  sortableKeyboardCoordinates
} from '@dnd-kit/sortable';

import { Layout, Button, Popover } from 'antd';
import { ToolOutlined, PicCenterOutlined, SubnodeOutlined, LeftOutlined, RightOutlined, DeleteOutlined } from '@ant-design/icons';
import { Canvas } from './Canvas';
import { ProgramBlock } from './ProgramBlock';
import { PrimitivesDrawer } from './PrimitivesDrawer'
import { ItemOverlay } from './Wrappers';
import useGuiStore from '../../stores/GuiStore';
import useEvdStore from '../../stores/EvdStore';

const getNewIndex = (active, over, overItems, overIndex, hierarchy) => {
  let newIndex;
  if (over.id in hierarchy) {
    newIndex = overItems.length + 1
  } else {
    const isBelowLastItem =
          over &&
          overIndex === overItems.length - 1 &&
          active.rect.current.translated &&
          active.rect.current.translated.offsetTop >
          over.rect.offsetTop + over.rect.height;

    const modifier = isBelowLastItem ? 1 : 0;

    newIndex = overIndex >= 0 ? overIndex + modifier : overItems.length + 1;
  }
  return newIndex;
}

export const ProgramEditor = (_) => {

  const [drawerExpanded, setDrawerExpanded] = useState(false);
  // const [parameterTabOpen, setParameterTabOpen] = useState(false)
  const [drawerOpen, setDrawerOpen] = useState(null);

  const [dragItem, setDragItem, clearDragItem] = useGuiStore(state => ([
    state.dragItem,
    state.setDragItem,
    state.clearDragItem
  ]));

  const {hierarchy,
         addItem, deleteItem,
         movePrimitiveId} = useEvdStore(state=>{
    let hierarchy = {};

    // first handle the program
    hierarchy[state.uuid] = state.primitiveIds;

    // next handle primitives that are hierarchical
    Object.keys(state.data.primitives).forEach(primitiveId=>{
      const primitive = state.data.primitives[primitiveId];
      if (primitive.type.includes('hierarchical')) {
        hierarchy[primitive.uuid] = primitive.primitiveIds;
      }
    });

    // // now skills (all are hierarchical)
    Object.keys(state.data.skills).forEach(skillId=>{
      const skill = state.data.skills[skillId];
      hierarchy[skill.uuid] = skill.primitiveIds;
    })

    // TODO: Add waypoint/trajectory containers
    return {hierarchy,
            addItem:state.addItem,
            deleteItem:state.deleteItem,
            movePrimitiveId:state.movePrimitiveId}
  })

  const sensors = useSensors(
    useSensor(PointerSensor),
    useSensor(KeyboardSensor, {
      coordinateGetter: sortableKeyboardCoordinates,
    })
  );

  const handleDragStart = (event) => {
    const { active } = event;
    const data = active.data.current;
    console.log(data);
    // If the action is a generic, add that data into the store.
    if (data.action.includes('generic')) {
      console.log(`adding ${data.itemType}`)
      addItem(data.itemType,data.record)
    }
    setDragItem(data);
    setDrawerOpen(null);
  }

  const handleDragOver = (event) => {
    const { active, over } = event;

    // This only really applies to sortables at this time.
    if (dragItem.action.includes('Drag')) {
      return;
    }

    // Ensure that the over item exists and has an id
    if (!over) {
      return;
    }

    // Get a bunch of info about the active item and what it is hovering over
    const activeId = dragItem.uuid;
    const activeContainer = dragItem.ancestors[0].uuid;
    const overData = over.data.current;
    let overId = null;
    let overIndex = null;
    let overContainer = null;
    let activeType = dragItem.record.type;
    overData.ancestors.forEach(ancestor=>{
      if (overId === null && ancestor.accepts.indexOf(activeType)>=0) {
        overId = ancestor.uuid;
        overIndex = over.data.current.idx;
        overContainer = ancestor.uuid;
      }
    })

    // If containers don't exist, cancel
    if (!activeContainer || !overContainer) {
      console.log('one container does not exist')
      console.log(`${activeContainer} ${overContainer} ${activeId} ${overId}`)
      return;
    } else {
      console.log('both containers exist')
      console.log(`${activeContainer} ${overContainer} ${activeId} ${overId}`)
    }

    const overItems = hierarchy[overContainer];

    const newIndex = getNewIndex(active, over, overItems, overIndex, hierarchy)

    movePrimitiveId(activeId,overContainer,newIndex)
  }

  const handleDragEnd = (event) => {
    const { active, over } = event;
    const overData = over.data.current;

    // Get a bunch of info about the active item and what it is hovering over
    const activeId = dragItem.uuid;
    const activeAction = dragItem.action;
    let overId = null;
    let overIndex = null;
    let overContainer = null;
    let activeType = dragItem.record.type;
    overData.ancestors.forEach(ancestor=>{
      if (overId === null && ancestor.accepts.indexOf(activeType)>=0) {
        overId = ancestor.uuid;
        overIndex = over.data.current.idx;
        overContainer = ancestor.uuid;
      }
    })

    if (!overContainer) {
      if (activeAction.includes('generic')) {
        deleteItem(activeType,activeId)
      }
      clearDragItem()
    }

    // Get the containers in
    if (activeType === 'primitive') {
      if (overId && activeId) {
        // Probably should add additional checks that the drop overId is compatible
        const overItems = hierarchy[overContainer];
        const newIndex = getNewIndex(active, over, overItems, overIndex, hierarchy)
        movePrimitiveId(activeId,overContainer,newIndex)
      }
    } else if (false /* This could be other drag item types (e.g. trajectories/waypoints) */) {

    }
    clearDragItem()
  }

  const handleDragCancel = (_) => {
    console.log('drag cancel')
    // If the drag action was a generic, remove it from the store
    if (dragItem.action.includes('generic')) {
      deleteItem(dragItem.itemType,dragItem.uuid)
    }
    clearDragItem();
  }

  const toggle = () => setDrawerExpanded(!drawerExpanded);

  let overlay = null;
  if (dragItem) {
    overlay = <ItemOverlay id={dragItem.uuid} itemType={dragItem.itemType}/>
  }

  return (
    <div style={{ width: '100%', height: '100%', display: 'flex' }}>
      <DndContext sensors={sensors}
        collisionDetection={closestCenter}
        onDragStart={handleDragStart}
        onDragOver={handleDragOver}
        onDragEnd={handleDragEnd}
        onDragCancel={handleDragCancel}
        >
        <Layout style={{ flex: 1 }}>
          <Layout.Sider collapsible collapsed={!drawerExpanded} trigger={null} style={{ align: 'left', display: 'flex', flexDirection: 'column', padding: 5 }}>
            <Button type='primary' block icon={drawerExpanded ? <LeftOutlined /> : <RightOutlined />} onClick={toggle} style={{ marginBottom: 5 }} />
            <Popover title='Parameters' trigger="click" placement='right'>
              <Button type='text' block icon={<SubnodeOutlined />} style={{ marginBottom: 5, alignItems: 'left' }}>
                {drawerExpanded && 'Parameters'}
              </Button>
            </Popover>
            <Popover title='Skills' trigger="click" placement='right'>
              <Button type='text' block icon={<ToolOutlined />} style={{ marginBottom: 5 }}>
                {drawerExpanded && 'Skills'}
              </Button>
            </Popover>
            <Popover
              title='Primitives'
              trigger="click"
              placement='right'
              visible={drawerOpen==='primitives'}
              content={<PrimitivesDrawer/>}
            >
              <Button type='text' block icon={<PicCenterOutlined />} style={{ marginBottom: 5 }} onClick={()=>setDrawerOpen(drawerOpen==='primitives'?null:'primitives')}>
                {drawerExpanded && 'Primitives'}
              </Button>
            </Popover>
            <Button danger hidden={dragItem===null||!dragItem.record.deleteable} type='outline' block icon={<DeleteOutlined />} style={{ marginBottom: 5, marginTop: 20 }}>
                {drawerExpanded && 'Delete'}
            </Button>
          </Layout.Sider>
          <Layout.Content style={{ height: 'calc(100vh - 115pt)', overflow: 'scroll' }}>

            <Canvas>
              <ProgramBlock />
            </Canvas>
          
          </Layout.Content>
        </Layout>
        <DragOverlay>
          {overlay}
        </DragOverlay>
      </DndContext>
    </div>
  )

}
