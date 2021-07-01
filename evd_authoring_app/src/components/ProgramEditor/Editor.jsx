import React, { useState } from 'react';
import { Layout, Button, Popover } from 'antd';
import { ToolOutlined, PicCenterOutlined, SubnodeOutlined, LeftOutlined, RightOutlined } from '@ant-design/icons';
import { Canvas } from './Canvas';
import { PrimitivesDrawer } from './PrimitivesDrawer'
import { DeleteZone } from './DeleteZone';

export const Editor = () => {

    const [drawerExpanded, setDrawerExpanded] = useState(false);
    
    const toggle = () => setDrawerExpanded(!drawerExpanded);
    
    return (
        <Layout style={{ flex: 1 }}>
          <Layout.Sider collapsible collapsed={!drawerExpanded} trigger={null} style={{ align: 'left', display: 'flex', flexDirection: 'column', padding: 5 }}>
            <Button type='primary' block icon={drawerExpanded ? <LeftOutlined /> : <RightOutlined />} onClick={toggle} style={{ marginBottom: 5 }} />
            <Popover title='Parameters' trigger="click" placement='right'>
              <Button type='text' block icon={<SubnodeOutlined />} style={{ marginBottom: 5, alignItems: 'left' }}>
                {drawerExpanded && 'Parameters'}
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