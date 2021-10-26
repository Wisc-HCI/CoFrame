import React from 'react';
import { Layout, Button, Row, Input ,Tooltip} from 'antd';
import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';
import Icon, {CloseOutlined, PlusOutlined} from '@ant-design/icons';
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
import { Canvas } from './Canvas';
import { primitiveTypes, fromPrimitiveTemplate } from '../../stores/templates';
import { DeleteZone } from './DeleteZone';
import {ReactComponent as LocationIcon} from '../CustomIcons/Location.svg';
import {ReactComponent as MachineIcon} from '../CustomIcons/Gear.svg';
import {ReactComponent as PrimitiveIcon} from '../CustomIcons/Primitive.svg';
import {ReactComponent as SkillIcon} from '../CustomIcons/Skill.svg';
import {ReactComponent as ThingIcon} from '../CustomIcons/Thing.svg';
import {ReactComponent as WaypointIcon} from '../CustomIcons/Waypoint.svg';
import {ReactComponent as ContainerIcon} from '../CustomIcons/Container.svg';
import { createWaypoint, createLocation, createMachine, createThing } from '../../stores/templates';
import { SortableContext } from '@dnd-kit/sortable';
import { generateUuid } from '../../stores/generateUuid';
import { fromContainerTemplate } from '../../stores/templates';
import { acceptLookup } from './acceptLookup';
import { Block } from './Blocks';

const skill2Call = (skill) => {
  let parameters = {skill_uuid:skill.uuid};
  skill.arguments.forEach(arg=>{parameters[arg.uuid] = null})

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

export const Editor = () => {

    const [
      activeDrawer, setActiveDrawer, searchTerm, 
      setSearchTerm, clearSearchTerm, addItem, setFocusItem, drawerValues] = useStore(store=>[
      store.activeDrawer,
      store.setActiveDrawer,
      store.searchTerm,
      store.setSearchTerm,
      store.clearSearchTerm,
      store.addItem,
      store.setFocusItem,
      {
        machines: Object.values(store.data).filter(v=>v.name.includes(store.searchTerm)&&v.type==='machine').map(item2block),
        locations: Object.values(store.data).filter(v=>v.name.includes(store.searchTerm)&&v.type==='location').map(item2block),
        waypoints: Object.values(store.data).filter(v=>v.name.includes(store.searchTerm)&&v.type==='waypoint').map(item2block),
        things: Object.values(store.data).filter(v=>v.name.includes(store.searchTerm)&&v.type==='thing').map(item2block),
        containers: [fromContainerTemplate('trajectory'),fromContainerTemplate('skill'),fromContainerTemplate('hierarchical')],
        skills: Object.values(store.data).filter(v=>v.name.includes(store.searchTerm)&&v.type==='trajectory').map(skill2Call),
        actions: primitiveTypes.map(type=>fromPrimitiveTemplate(type))
      }
    ],shallow);

    const drawerStyle = useSpring({width: activeDrawer ? 270 : 0, padding: activeDrawer ? 5 : 0, config:config.stiff});

    const drawers = {
      machines: {
        title: "Machines",
        icon: <Icon component={MachineIcon}/>,
        // drawer: <UUIDDrawer itemType='machine'/>,
        values: drawerValues.machines,
        addFn: (mesh)=>{
          const newMachine = createMachine(mesh);
          addItem('machine',newMachine);
          setFocusItem('machine',newMachine.uuid);
        }
      },
      locations: {
        title: "Locations",
        icon: <Icon component={LocationIcon}/>,
        // drawer: <UUIDDrawer itemType='location'/>,
        values: drawerValues.locations,
        addFn: ()=>{
          const newLocation = createLocation();
          addItem('location',newLocation);
          setFocusItem('location',newLocation.uuid);
        }
      },
      waypoints: {
        title: "Waypoints",
        icon: <Icon component={WaypointIcon}/>,
        // drawer: <UUIDDrawer itemType='waypoint'/>,
        values: drawerValues.waypoints,
        addFn: ()=>{
          const newWaypoint = createWaypoint();
          addItem('waypoint',newWaypoint);
          setFocusItem('waypoint',newWaypoint.uuid);
        }
      },
      things: {
        title: "Things",
        icon: <Icon component={ThingIcon}/>,
        // drawer: <UUIDDrawer itemType='thing'/>,
        values: drawerValues.things,
        addFn: (thingTypeUuid)=>{
          const newThing = createThing(thingTypeUuid);
          addItem('thing',newThing);
          setFocusItem('thing',newThing.uuid);
        }
      },
      containers: {
        title: "Containers",
        icon: <Icon component={ContainerIcon}/>,
        // drawer: <ContainersDrawer/>
        values: drawerValues.containers,
      },
      skills: {
        title: "Skills",
        icon: <Icon component={SkillIcon}/>,
        // drawer: <SkillCallDrawer/>,
        values: drawerValues.skills,
      },
      actions: {
        title: "Actions",
        icon: <Icon component={PrimitiveIcon}/>,
        // drawer: <PrimitivesDrawer/>,
        values: drawerValues.actions,

      }
    }

    const ancestors = [
      {uuid:'drawer',...acceptLookup.drawer.default}
    ];
    
    return (
      <SortableContext items={activeDrawer?drawers[activeDrawer].values.map(v=>v.uuid):[]}>
        <Layout style={{ fontSize: 20, height: 'calc(100vh - 113pt)' }}>
          <Layout.Sider collapsed collapsible trigger={null} style={{align: 'left', display: 'flex', flexDirection: 'column', padding: 5, height: 'calc(100vh - 113pt)' }}>
            {Object.keys(drawers).map(drawerKey=>(
              <Tooltip key={drawerKey} placement="right" title = {drawers[drawerKey].title}>
              <Button 
                type={activeDrawer === drawerKey ? 'primary' : 'text'} 
                block 
                icon={drawers[drawerKey].icon} 
                onClick={()=>{ clearSearchTerm(); drawerKey===activeDrawer ? setActiveDrawer(null) : setActiveDrawer(drawerKey)}}
                style={{ marginBottom: 5, alignItems: 'left' }}/>
               </Tooltip>
            ))}
          </Layout.Sider>
          <animated.div onDrag={()=>setActiveDrawer(null)} style={{...drawerStyle, align: 'left', backgroundColor: '#2f2f2f', fontSize: 14, height: 'calc(100vh - 113pt)'}}>
             {activeDrawer && (
               <React.Fragment>
                <Row align='middle' justify='space-between' style={{padding:5,marginLeft:5}}>
                  <span style={{color:'white',padding:3}}>{drawers[activeDrawer].title}</span>
                  <Button
                      type='outline'
                      onClick={drawers[activeDrawer].addFn}
                      icon={<PlusOutlined />}
                    />
                </Row>
                <Row style={{backgroundColor:'#f1f1f110', width:'100%'}}>
                    <Input
                      value={searchTerm} 
                      placeholder="Search..." 
                      onChange={(e)=>setSearchTerm(e.target.value)} 
                      addonAfter={<CloseOutlined onClick={clearSearchTerm}/>}
                      style={{ maxWidth: 300, minWidth: 100, display: "block" }} />
                  </Row>
                <div style={{width:'100%',marginTop:10, overflowY: 'scroll',height: 'calc(100vh - 183pt)'}}>
                    {drawers[activeDrawer].values.map(v=>(
                      <Block key={v.uuid} ancestors={ancestors} staticData={v} context={{}} dragDisabled={false}/>
                    ))}
                </div>
                
               </React.Fragment>
             )}
          </animated.div>
          <Layout.Content style={{ height: 'calc(100vh - 113pt)'}}>
            <Canvas/>
            <DeleteZone/>
          </Layout.Content>
        </Layout>
      </SortableContext>
    )
}