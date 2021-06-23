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
  arrayMove,
  sortableKeyboardCoordinates
} from '@dnd-kit/sortable';

import { Layout, Button, Popover } from 'antd';
import { ToolOutlined, PicCenterOutlined, SubnodeOutlined, LeftOutlined, RightOutlined, DeleteOutlined } from '@ant-design/icons';
import { Grid } from './Grid';
import { ProgramBlock } from './ProgramBlock';
import { PrimitivesDrawer } from './PrimitivesDrawer'
import { GenericOverlay, ItemOverlay } from './Wrappers';
import useGuiStore from '../../stores/GuiStore';
import useEvdStore from '../../stores/EvdStore';

export const ProgramEditor = (_) => {

  const [drawerExpanded, setDrawerExpanded] = useState(false);
  // const [parameterTabOpen, setParameterTabOpen] = useState(false)
  const [drawerOpen, setDrawerOpen] = useState(null);

  const [dragItem, setDragItem, clearDragItem] = useGuiStore(state => ([
    state.dragItem,
    state.setDragItem,
    state.clearDragItem
  ]));

  const {containers, hierarchy, 
         addItem, deleteItem, 
         setPrimitiveIds, insertPrimitiveId, 
         deletePrimitiveId, movePrimitiveId} = useEvdStore(state=>{
    let containers = {};
    let hierarchy = {};

    // first handle the program
    state.primitiveIds.forEach(id=>containers[id]=state.uuid);
    hierarchy[state.uuid] = state.primitiveIds;

    // next handle primitives that are hierarchical
    Object.keys(state.data.primitives).forEach(primitiveId=>{
      const primitive = state.data.primitives[primitiveId];
      if (primitive.type.includes('hierarchical')) {
        primitive.primitiveIds.forEach(id=>containers[id]=primitive.uuid);
        hierarchy[primitive.uuid] = primitive.primitiveIds;
      }
    });

    // // now skills (all are hierarchical)
    Object.keys(state.data.skills).forEach(skillId=>{
      const skill = state.data.skills[skillId];
      skill.primitiveIds.forEach(id=>containers[id]=skill.uuid);
      hierarchy[skill.uuid] = skill.primitiveIds;
    })

    // TODO: Add waypoint/trajectory containers
    return {containers, hierarchy, 
            addItem:state.addItem, 
            deleteItem:state.deleteItem,
            setPrimitiveIds:state.setPrimitiveIds, 
            insertPrimitiveId:state.insertPrimitiveId, 
            deletePrimitiveId:state.deletePrimitiveId,
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
    // If the action is a generic, add that data into the store.
    if (data.action.includes('generic')) {
      console.log(`adding ${data.itemType}`)
      addItem(data.itemType,data.initial)
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
    if (!over.id) {
      return;
    }

    const activeId = dragItem.uuid;
    const overId = over.id;

    // Get the containers for each
    let activeContainer = containers[activeId];
    let overContainer = containers[overId];
    if (!overContainer && overId in Object.keys(hierarchy)) {
      console.log('fallback on container')
      overContainer = overId
    }

    if (!activeContainer && active.data.current.source === 'drawer') {
      activeContainer = 'drawer';
    }

    // If containers don't exist, cancel
    if (!activeContainer || !overContainer) {
      console.log('one container does not exist')
      return;
    }

    const overItems = hierarchy[overContainer];
    const overIndex = overItems.indexOf(overId);

    let newIndex;
    if (overId in hierarchy) {
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

    movePrimitiveId(activeId,overContainer,newIndex)
  }

  const handleDragEnd = (event) => {
    const { over } = event;
    const activeId = dragItem.uuid;
    const activeType = dragItem.itemType;
    const activeAction = dragItem.action;
    const overId = over.id;
    let overContainer = containers[overId];
    if (!overContainer && overId in Object.keys(hierarchy)) {
      console.log('fallback on container')
      overContainer = overId
    }

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
        const newIndex = hierarchy[overContainer].indexOf(overId);
        movePrimitiveId(activeId,overContainer,newIndex)
      }
    } else if (false /* This could be other drag item types (e.g. trajectories/waypoints) */) {
    
    }
    clearDragItem()
  }

  const handleDragCancel = (event) => {
    const { active } = event;
    const data = active.data.current;
    // If the drag action was a generic, remove it from the store
    if (data.action.includes('generic')) {
      deleteItem(data.itemType,dragItem.uuid)
    }
    clearDragItem();
  }

  const toggle = () => setDrawerExpanded(!drawerExpanded);

  let overlay = null;
  if (dragItem) {
    console.log(dragItem.uuid)
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
            <Button danger hidden={dragItem===null} type='outline' block icon={<DeleteOutlined />} style={{ marginBottom: 5, marginTop: 20 }}>
                {drawerExpanded && 'Delete'}
            </Button>
          </Layout.Sider>
          <Layout.Content style={{ height: 'calc(100vh - 115pt)', overflow: 'scroll' }}>
            <Grid>
              <ProgramBlock />
            </Grid>
          </Layout.Content>
        </Layout>
        <DragOverlay>
          {overlay}
        </DragOverlay>
      </DndContext>
    </div>
  )

}