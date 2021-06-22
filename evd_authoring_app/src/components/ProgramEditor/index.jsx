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

  const {containers, hierarchy, addItem, setPrimitiveIds, insertPrimitiveId, deletePrimitiveId} = useEvdStore(state=>{
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
            setPrimitiveIds:state.setPrimitiveIds, 
            insertPrimitiveId:state.insertPrimitiveId, 
            deletePrimitiveId:state.deletePrimitiveId}
  })

  const sensors = useSensors(
    useSensor(PointerSensor),
    useSensor(KeyboardSensor, {
      coordinateGetter: sortableKeyboardCoordinates,
    })
  );

  const handleDragEnd = (event) => {
    const { active, over } = event;

    if (active.data.current.action === 'itemSort') {
      if (active.data.current.itemType === 'primitive') {
        if (containers[over.id] !== containers[active.id]) {
          // If the item is moved from outside the current list
          // Remove from the old list
          const oldContainer = containers[active.id];
          const oldIndex = hierarchy[oldContainer].indexOf(active.id);
          deletePrimitiveId(oldContainer,oldIndex)
          // Add the new list
          const newContainer = containers[over.id];
          const newIndex = hierarchy[newContainer].indexOf(over.id);
          insertPrimitiveId(newIndex,newContainer)
        } else {
          // If the item is moved from within the current list
          const container = containers[active.id];
          const oldIndex = hierarchy[container].indexOf(active.id);
          const newIndex = hierarchy[container].indexOf(over.id);
          const newIds = arrayMove(hierarchy[container], oldIndex, newIndex);
          setPrimitiveIds(newIds, container);
        }
      } else if (false /* This could be other drag item types (e.g. trajectories/waypoints) */) {
      
      }
    } else if (active.data.current.action === 'genericSort') {
      if (active.data.current.itemType === 'primitive') {
        // If copying, there should be a field 'primitiveData' attached to the current data
        const newPrimitiveData = active.data.current.initial;
        const container = containers[over.id];
        addItem('primitive', newPrimitiveData);
        insertPrimitiveId(newPrimitiveData.uuid, containers[over.id], hierarchy[container].indexOf(over.id))
      } else if (false /* This could be other drag item types (e.g. trajectories/waypoints) */) {
      
      }
    }
    clearDragItem()
  }

  const handleDragStart = (event) => {
    const { active } = event;
    setDragItem(active.data.current);
    setDrawerOpen(null);
  }

  const toggle = () => setDrawerExpanded(!drawerExpanded);

  let overlay = null;
  if (dragItem && dragItem.action === 'genericSort') {
    overlay = <GenericOverlay type={dragItem.type} itemType={dragItem.itemType}/>
  } else if (dragItem && dragItem.action === 'itemSort') {
    overlay = <ItemOverlay id={dragItem.uuid} itemType={dragItem.itemType}/>
  }

  return (
    <div style={{ width: '100%', height: '100%', display: 'flex' }}>
      <DndContext sensors={sensors}
        collisionDetection={closestCenter}
        onDragEnd={handleDragEnd}
        onDragStart={handleDragStart}>
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