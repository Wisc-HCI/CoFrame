import React from 'react';
import { Layout, Button, Row, Input ,Tooltip} from 'antd';
import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';
import Icon, {CloseOutlined, PlusOutlined} from '@ant-design/icons';
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
import { Canvas } from './Canvas';
import { PrimitivesDrawer } from './Drawers/PrimitivesDrawer';
import { ContainersDrawer } from './Drawers/ContainersDrawer';
import { UUIDDrawer } from './Drawers/UUIDDrawer';
import { SkillCallDrawer } from './Drawers/SkillCallDrawer';
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

const SEARCHABLE = ['machines','locations','waypoints','placeholders']

export const Editor = () => {

    const [
      activeDrawer, setActiveDrawer, searchTerm, 
      setSearchTerm, clearSearchTerm, addItem, setFocusItem] = useStore(store=>[
      store.activeDrawer,
      store.setActiveDrawer,
      store.searchTerm,
      store.setSearchTerm,
      store.clearSearchTerm,
      store.addItem,
      store.setFocusItem
    ],shallow);

    const drawerStyle = useSpring({width: activeDrawer ? 270 : 0, padding: activeDrawer ? 5 : 0, config:config.stiff});

    const drawers = {
      machines: {
        title: "Machines",
        icon: <Icon component={MachineIcon}/>,
        drawer: <UUIDDrawer itemType='machine'/>,
        addFn: (mesh)=>{
          const newMachine = createMachine(mesh);
          addItem('machine',newMachine);
          setFocusItem('machine',newMachine.uuid);
        }
      },
      locations: {
        title: "Locations",
        icon: <Icon component={LocationIcon}/>,
        drawer: <UUIDDrawer itemType='location'/>,
        addFn: ()=>{
          const newLocation = createLocation();
          addItem('location',newLocation);
          setFocusItem('location',newLocation.uuid);
        }
      },
      waypoints: {
        title: "Waypoints",
        icon: <Icon component={WaypointIcon}/>,
        drawer: <UUIDDrawer itemType='waypoint'/>,
        addFn: ()=>{
          const newWaypoint = createWaypoint();
          addItem('waypoint',newWaypoint);
          setFocusItem('waypoint',newWaypoint.uuid);
        }
      },
      placeholders: {
        title: "Things",
        icon: <Icon component={ThingIcon}/>,
        drawer: <UUIDDrawer itemType='thing'/>,
        addFn: (thingTypeUuid)=>{
          const newThing = createThing(thingTypeUuid);
          addItem('thing',newThing);
          setFocusItem('thing',newThing.uuid);
        }
      },
      containers: {
        title: "Containers",
        icon: <Icon component={ContainerIcon}/>,
        drawer: <ContainersDrawer/>
      },
      skills: {
        title: "Skills",
        icon: <Icon component={SkillIcon}/>,
        drawer: <SkillCallDrawer/>
      },
      actions: {
        title: "Actions",
        icon: <Icon component={PrimitiveIcon}/>,
        drawer: <PrimitivesDrawer/>
      }
    }
    
    return (
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
                  {SEARCHABLE.indexOf(activeDrawer) >= 0 && (
                    <Button
                      type='outline'
                      onClick={drawers[activeDrawer].addFn}
                      icon={<PlusOutlined />}
                    />
                  )}
                </Row>
                {SEARCHABLE.indexOf(activeDrawer) >= 0 && (
                  <Row style={{backgroundColor:'#f1f1f110', width:'100%'}}>
                    <Input
                      value={searchTerm} 
                      placeholder="Search..." 
                      onChange={(e)=>setSearchTerm(e.target.value)} 
                      addonAfter={<CloseOutlined onClick={clearSearchTerm}/>}
                      style={{ maxWidth: 300, minWidth: 100, display: "block" }} />
                  </Row>
                )}
                <div style={{width:'100%',marginTop:10, overflowY: 'scroll',height: SEARCHABLE.indexOf(activeDrawer) === -1 ? 'calc(100vh - 155pt)' : 'calc(100vh - 183pt)'}}>
                  <SortableContext>
                    {drawers[activeDrawer].drawer}
                  </SortableContext>
                </div>
                
               </React.Fragment>
             )}
          </animated.div>
          <Layout.Content style={{ height: 'calc(100vh - 113pt)'}}>
            <Canvas/>
            <DeleteZone/>
          </Layout.Content>
        </Layout>
    )
}