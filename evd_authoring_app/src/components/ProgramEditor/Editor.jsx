import React, { useState } from 'react';
import { Layout, Button, Popover } from 'antd';
import { ToolOutlined, PicCenterOutlined, SubnodeOutlined, LeftOutlined, RightOutlined } from '@ant-design/icons';
import { Canvas } from './Canvas';
import { PrimitivesDrawer } from './PrimitivesDrawer';
import { UUIDDrawer } from './UUIDDrawer';
import { DeleteZone } from './DeleteZone';

export const Editor = () => {

    const [drawerExpanded, setDrawerExpanded] = useState(false);
    
    const toggle = () => setDrawerExpanded(!drawerExpanded);
    
    return (
        <Layout style={{ flex: 1 }}>
          <Layout.Sider collapsible collapsed={!drawerExpanded} trigger={null} style={{ align: 'left', display: 'flex', flexDirection: 'column', padding: 5 }}>
            <Button type='primary' block icon={drawerExpanded ? <LeftOutlined /> : <RightOutlined />} onClick={toggle} style={{ marginBottom: 5 }} />
            <Popover 
                title='Machines'
                placement='right'
                content={<UUIDDrawer itemType='machine'/>}
              >
              <Button type='text' block icon={<SubnodeOutlined />} style={{ marginBottom: 5, alignItems: 'left' }}>
                {drawerExpanded && 'Machines'}
              </Button>
            </Popover>
            <Popover 
                title='Locations'
                placement='right'
                content={<UUIDDrawer itemType='location'/>}
              >
              <Button type='text' block icon={<SubnodeOutlined />} style={{ marginBottom: 5, alignItems: 'left' }}>
                {drawerExpanded && 'Locations'}
              </Button>
            </Popover>
            <Popover 
                title='Things'
                placement='right'
                content={<UUIDDrawer itemType='thing'/>}
              >
              <Button type='text' block icon={<SubnodeOutlined />} style={{ marginBottom: 5, alignItems: 'left' }}>
                {drawerExpanded && 'Things'}
              </Button>
            </Popover>
            <Popover 
                title='Waypoints'
                placement='right'
                content={<UUIDDrawer itemType='waypoint'/>}
              >
              <Button type='text' block icon={<SubnodeOutlined />} style={{ marginBottom: 5, alignItems: 'left' }}>
                {drawerExpanded && 'Waypoints'}
              </Button>
            </Popover>
            <Popover 
                title='Macros'
                placement='right'>
              <Button type='text' block icon={<ToolOutlined />} style={{ marginBottom: 5 }}>
                {drawerExpanded && 'Skills'}
              </Button>
            </Popover>
            <Popover
                title='Actions'
                placement='right'
                content={<PrimitivesDrawer/>}
            >
              <Button type='text' block icon={<PicCenterOutlined />} style={{ marginBottom: 5 }}>
                {drawerExpanded && 'Primitives'}
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