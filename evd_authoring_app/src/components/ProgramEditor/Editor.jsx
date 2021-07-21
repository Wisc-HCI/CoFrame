import React, { useState } from 'react';
import { Layout, Button, Popover } from 'antd';
import Icon, { LeftOutlined, RightOutlined } from '@ant-design/icons';
import { Canvas } from './Canvas';
import { PrimitivesDrawer } from './PrimitivesDrawer';
import { UUIDDrawer } from './UUIDDrawer';
import { DeleteZone } from './DeleteZone';
import {ReactComponent as LocationIcon} from '../CustomIcons/Location.svg';
import {ReactComponent as MachineIcon} from '../CustomIcons/Gear.svg';
import {ReactComponent as PrimitiveIcon} from '../CustomIcons/Primitive.svg';
import {ReactComponent as SkillIcon} from '../CustomIcons/Skill.svg';
import {ReactComponent as ThingIcon} from '../CustomIcons/Thing.svg';
import {ReactComponent as WaypointIcon} from '../CustomIcons/Waypoint.svg';

export const Editor = () => {

    const [drawerExpanded, setDrawerExpanded] = useState(false);
    
    const toggle = () => setDrawerExpanded(!drawerExpanded);
    
    return (
        <Layout style={{ flex: 1 }}>
          <Layout.Sider collapsible collapsed={!drawerExpanded} trigger={null} style={{ align: 'left', display: 'flex', flexDirection: 'column', padding: 5 }}>
            <Button type='primary' block icon={drawerExpanded ? <LeftOutlined /> : <RightOutlined />} onClick={toggle} style={{ marginBottom: 5 }} />
            <Popover 
                title={<span><Icon style={{marginRight:10}} component={MachineIcon}/>Machines</span>}
                placement='right'
                content={<UUIDDrawer itemType='machine'/>}
              >
              <Button type='text' block icon={<Icon component={MachineIcon}/>} style={{ marginBottom: 5, alignItems: 'left' }}>
                {drawerExpanded && 'Machines'}
              </Button>
            </Popover>
            <Popover 
                title={<span><Icon style={{marginRight:10}} component={LocationIcon}/>Locations</span>}
                placement='right'
                content={<UUIDDrawer itemType='location'/>}
              >
              <Button type='text' block icon={<Icon component={LocationIcon}/>} style={{ marginBottom: 5, alignItems: 'left' }}>
                {drawerExpanded && 'Locations'}
              </Button>
            </Popover>
            <Popover 
                title={<span><Icon style={{marginRight:10}} component={WaypointIcon}/>Waypoints</span>}
                placement='right'
                content={<UUIDDrawer itemType='waypoint'/>}
              >
              <Button type='text' block icon={<Icon component={WaypointIcon}/>} style={{ marginBottom: 5, alignItems: 'left' }}>
                {drawerExpanded && 'Waypoints'}
              </Button>
            </Popover>
            <Popover 
                title={<span><Icon style={{marginRight:10}} component={ThingIcon}/>Things</span>}
                placement='right'
                content={<UUIDDrawer itemType='thing'/>}
              >
              <Button type='text' block icon={<Icon component={ThingIcon}/>} style={{ marginBottom: 5, alignItems: 'left' }}>
                {drawerExpanded && 'Things'}
              </Button>
            </Popover>
            <Popover 
                title={<span><Icon style={{marginRight:10}} component={SkillIcon}/>Skills</span>}
                placement='right'>
              <Button type='text' block icon={<Icon component={SkillIcon}/>} style={{ marginBottom: 5 }}>
                {drawerExpanded && 'Macros'}
              </Button>
            </Popover>
            <Popover 
                title={<span><Icon style={{marginRight:10}} component={PrimitiveIcon}/>Actions</span>}
                placement='right'
                content={<PrimitivesDrawer/>}
              >
              <Button type='text' block icon={<Icon component={PrimitiveIcon}/>} style={{ marginBottom: 5 }}>
                {drawerExpanded && 'Actions'}
              </Button>
            </Popover>
          </Layout.Sider>
          <Layout.Content style={{ height: 'calc(100vh - 115pt)', overflow: 'scroll' }}>
            <Canvas/>
            <DeleteZone/>
          </Layout.Content>
        </Layout>
    )
}