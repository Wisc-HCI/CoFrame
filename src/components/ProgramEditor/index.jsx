import React, { useState, useRef } from 'react';
import { Button, Row, Input, Tooltip } from 'antd';
import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';
import Icon, { CloseOutlined, PlusOutlined } from '@ant-design/icons';
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
import { Canvas } from './Canvas';
import { primitiveTypes, fromPrimitiveTemplate } from '../../stores/templates';
import { DeleteZone } from './DeleteZone';
import { ReactComponent as LocationIcon } from '../CustomIcons/Location.svg';
import { ReactComponent as MachineIcon } from '../CustomIcons/Gear.svg';
import { ReactComponent as PrimitiveIcon } from '../CustomIcons/Primitive.svg';
import { ReactComponent as SkillIcon } from '../CustomIcons/Skill.svg';
import { ReactComponent as ThingIcon } from '../CustomIcons/Thing.svg';
import { ReactComponent as WaypointIcon } from '../CustomIcons/Waypoint.svg';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg';
import { createWaypoint, createLocation, createMachine, createThing } from '../../stores/templates';
// import { SortableContext } from '@dnd-kit/sortable';
import { generateUuid } from '../../stores/generateUuid';
import { fromContainerTemplate } from '../../stores/templates';
import { acceptLookup } from './acceptLookup';
import { Block } from './Blocks';
// import { DndProvider } from 'react-dnd';
import {
  DndContext,
  DragOverlay,
  KeyboardSensor,
  PointerSensor,
  useSensor,
  useSensors,
} from '@dnd-kit/core';
import { thresholdedClosestCorners } from '../../stores/helpers';
import {
  sortableKeyboardCoordinates
} from '@dnd-kit/sortable';
// import { HTML5Backend } from 'react-dnd-html5-backend';
// import { MultiBackend } from 'react-dnd-multi-backend'
import { DisplayBlock } from './Blocks/DisplayBlock';
import {
  restrictToWindowEdges,
} from '@dnd-kit/modifiers';
import { ReactFlowProvider } from 'react-flow-renderer';
import useMeasure from 'react-use-measure';

const skill2Call = (skill) => {
  let parameters = { skill_uuid: skill.uuid };
  skill.arguments.forEach(arg => { parameters[arg.uuid] = null })

  return {
    type: 'skill-call',
    uuid: generateUuid('skill-call'),
    name: `Execute Skill: ${skill.name}`,
    readonly: false,
    description: '',
    parameters
  }
}

const item2block = (item) => {
  let type = `uuid-${item.type}`;
  return {
    type,
    uuid: generateUuid(type),
    ref: item.uuid,
    readonly: false,
  }
}

export const ProgramEditor = (_) => {

  const sensors = useSensors(
    useSensor(PointerSensor),
    useSensor(KeyboardSensor, {
      coordinateGetter: sortableKeyboardCoordinates,
    })
  );

  const [flowInstance, setFlowInstance] = useState(null);
  const flowWrapper = useRef(null);

  const [
    editorTransform,
    dragData, setDragData, setDropData, clearDragDrop,
    activeDrawer, setActiveDrawer, searchTerm,
    setSearchTerm, clearSearchTerm, addItem, setFocusItem, drawerValues] = useStore(store => [
      store.editorTransform,
      store.dragData,
      store.setDragData,
      store.setDropData,
      store.clearDragDrop,
      store.activeDrawer,
      store.setActiveDrawer,
      store.searchTerm,
      store.setSearchTerm,
      store.clearSearchTerm,
      store.addItem,
      store.setFocusItem,
      {
        machines: Object.values(store.data).filter(v => v.name.includes(store.searchTerm) && v.type === 'machine').map(item2block),
        locations: Object.values(store.data).filter(v => v.name.includes(store.searchTerm) && v.type === 'location').map(item2block),
        waypoints: Object.values(store.data).filter(v => v.name.includes(store.searchTerm) && v.type === 'waypoint').map(item2block),
        things: Object.values(store.data).filter(v => v.name.includes(store.searchTerm) && v.type === 'thing').map(item2block),
        containers: [fromContainerTemplate('trajectory'), fromContainerTemplate('skill'), fromContainerTemplate('hierarchical')],
        skills: Object.values(store.data).filter(v => v.name.includes(store.searchTerm) && v.type === 'trajectory').map(skill2Call),
        actions: primitiveTypes.map(type => fromPrimitiveTemplate(type))
      }
    ], shallow);

  // console.log(editorTransform)

  const handleDragStart = (event) => {
    const { active } = event;
    console.log(active.data.current)
    if (active) {
      console.log(active.data.current)
      setDragData(active.data.current)
      setActiveDrawer(null)
    }
  }

  const handleDragOver = (event) => {
    const { active, over } = event;
    console.log({ active, over })
    if (over) {
      setDropData(over.data.current)
    } else {
      setDropData(null)
    }
  }
  
  const handleDragEnd = (event) => {
    const { active, over } = event;
    console.log(event);
    if (over && over.id === 'grid' && ['program', 'skill'].includes(active.data.current.type)) {
      const canvasBounds = flowWrapper.current.getBoundingClientRect();
      console.log(event)
      // const position = flowInstance.project({
      //   x: 
      // })
    }
    if (over && active.id !== over.id) {
      console.log({ active, over })
    }
    clearDragDrop()
  }

  // const [ref, {width}] = useMeasure();

  const drawerStyle = useSpring({ width: activeDrawer ? '205pt' : '0pt', config: config.stiff });
  const drawers = {
    machines: {
      title: "Machines",
      icon: <Icon style={{fontSize:15}} component={MachineIcon} />,
      // drawer: <UUIDDrawer itemType='machine'/>,
      values: drawerValues.machines,
      addFn: (mesh) => {
        const newMachine = createMachine(mesh);
        addItem('machine', newMachine);
        setFocusItem('machine', newMachine.uuid);
      }
    },
    locations: {
      title: "Locations",
      icon: <Icon style={{fontSize:15}} component={LocationIcon} />,
      // drawer: <UUIDDrawer itemType='location'/>,
      values: drawerValues.locations,
      addFn: () => {
        const newLocation = createLocation();
        addItem('location', newLocation);
        setFocusItem('location', newLocation.uuid);
      }
    },
    waypoints: {
      title: "Waypoints",
      icon: <Icon style={{fontSize:15}} component={WaypointIcon} />,
      // drawer: <UUIDDrawer itemType='waypoint'/>,
      values: drawerValues.waypoints,
      addFn: () => {
        const newWaypoint = createWaypoint();
        addItem('waypoint', newWaypoint);
        setFocusItem('waypoint', newWaypoint.uuid);
      }
    },
    things: {
      title: "Things",
      icon: <Icon style={{fontSize:15}} component={ThingIcon} />,
      // drawer: <UUIDDrawer itemType='thing'/>,
      values: drawerValues.things,
      addFn: (thingTypeUuid) => {
        const newThing = createThing(thingTypeUuid);
        addItem('thing', newThing);
        setFocusItem('thing', newThing.uuid);
      }
    },
    containers: {
      title: "Containers",
      icon: <Icon style={{fontSize:15}} component={ContainerIcon} />,
      // drawer: <ContainersDrawer/>
      values: drawerValues.containers,
    },
    skills: {
      title: "Skills",
      icon: <Icon style={{fontSize:15}} component={SkillIcon} />,
      // drawer: <SkillCallDrawer/>,
      values: drawerValues.skills,
    },
    actions: {
      title: "Actions",
      icon: <Icon style={{fontSize:15}} component={PrimitiveIcon} />,
      // drawer: <PrimitivesDrawer/>,
      values: drawerValues.actions,

    }
  }

  const ancestors = [
    { uuid: 'drawer', ...acceptLookup.drawer.default }
  ];

  return (

    
      <DndContext
        sensors={sensors}
        collisionDetection={({collisionRect,droppableContainers})=>thresholdedClosestCorners({collisionRect,droppableContainers,editorTransform})}
        onDragEnd={handleDragEnd}
        onDragStart={handleDragStart}
        onDragOver={handleDragOver}
        modifiers={[restrictToWindowEdges]}
      >
        <div style={{ fontSize: 20, height: '100%', display: 'flex' }}>
          <div style={{ align: 'left', display: 'flex', flexDirection: 'column', padding: 5, height: 'calc(100vh - 108pt)', width: 60, backgroundColor:'rgba(31,31,31,0.4)'}}>
            {Object.keys(drawers).map(drawerKey => (
              <Tooltip key={drawerKey} placement="right" title={drawers[drawerKey]?.title}>
                <Button
                  type={activeDrawer === drawerKey ? 'primary' : 'text'}
                  block
                  icon={drawers[drawerKey].icon}
                  onClick={() => { clearSearchTerm(); drawerKey === activeDrawer ? setActiveDrawer(null) : setActiveDrawer(drawerKey) }}
                  style={{ marginBottom: 5, alignItems: 'left' }} />
              </Tooltip>
            ))}
          </div>
          <animated.div style={{ ...drawerStyle, align: 'left', backgroundColor: '#2f2f2f', fontSize: 14, height: 'calc(100vh - 108pt)' }}>
            {activeDrawer && (
              <React.Fragment>
                <Row align='middle' justify='space-between' style={{ padding: 5, marginLeft: 5 }}>
                  <span style={{ color: 'white', padding: 3 }}>{drawers[activeDrawer]?.title}</span>
                  <Button
                    type='outline'
                    onClick={drawers[activeDrawer].addFn}
                    icon={<PlusOutlined />}
                  />
                </Row>
                <Row style={{ backgroundColor: '#f1f1f110', width: '100%' }}>
                  <Input
                    value={searchTerm}
                    placeholder="Search..."
                    onChange={(e) => setSearchTerm(e.target.value)}
                    addonAfter={<CloseOutlined onClick={clearSearchTerm} />}
                    style={{ maxWidth: 300, minWidth: 100, display: "block" }} />
                </Row>
                <div style={{ width: '100%', marginTop: 10, height: 'calc(100vh - 178pt)', overflowY:'scroll'}}>
                  {drawers[activeDrawer].values.map((v,i) => (
                    <div key={v.uuid} style={{ marginBottom: 5, marginLeft: 5, marginRight: 5, marginTop: i===0?5:0 }}>
                      <Block ancestors={ancestors} staticData={v} context={{}} dragDisabled={false} />
                    </div>
                  ))}
                </div>

              </React.Fragment>
            )}
          </animated.div>
          {/* <div style={{flex:1,backgroundColor:'red',height:'100%'}}>

          </div> */}
          <div style={{height: 'calc(100vh - 108pt)',flex:1}}>
            <ReactFlowProvider>
              <div ref={flowWrapper} style={{ height: '100%', width: '100%' }}>
                <Canvas setFlowInstance={setFlowInstance} />
                <DeleteZone />
              </div>
            </ReactFlowProvider>
          </div>
        </div>
        <DragOverlay>
          {dragData && (
            <DisplayBlock ancestors={dragData.ancestors} staticData={dragData.data} context={dragData.context} dragDisabled={true} />
          )}
        </DragOverlay>
      </DndContext>
  )

}
