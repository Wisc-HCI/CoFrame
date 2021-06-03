import React from 'react';

import { Tabs, Card, Button } from 'antd';
import { PlusOutlined } from '@ant-design/icons';

import { ItemList } from './Tabs/ItemList';
// import { WaypointList } from './Tabs/Waypoints';
// import { RegionList } from './Tabs/Regions';
// import { MachineList } from './Tabs/Machines';
// import { ThingList } from './Tabs/Things';
// import { ThingTypeList } from './Tabs/ThingTypes';


import useGuiStore from '../../stores/GuiStore';

// import './index.css'


export function SetupEditor(_) {

    const {primaryColor, setupTab, setSetupTab} = useGuiStore(state=>({
        primaryColor:state.primaryColor,
        setupTab:state.setupTab,
        setSetupTab:state.setSetupTab
    }));

    const tabs = [
        {
            key:'locations',
            name:'Locations',
            type:'location',
            title: (item)=> `${item.name}`,
            description: (item)=> `Info about ${item.name}`
        },
        {
            key:'machines',
            name:'Machines',
            type:'machine',
            title: (item)=> `${item.name}`,
            description: (item)=> `Info about ${item.name}`
        },
        {
            key:'waypoints',
            name:'Waypoints',
            type:'waypoint',
            title: (item)=> `${item.name}`,
            description: (item)=> `Info about ${item.name}`
        },
        {
            key:'things',
            name:'Things',
            type:'thingType',
            title: (item)=> `${item.name}`,
            description: (item)=> `Info about ${item.name}`
        }
    ]

    return (

        <Tabs
            tabPosition='left'
            style={{display:'flex',flex:1}}
            activeKey={setupTab}
            onChange={setSetupTab}
        >
            {tabs.map(tab=>(
                <Tabs.TabPane
                    key={tab.key}
                    tab={<span style={tab.key === setupTab ? { color: primaryColor } : {}}>{tab.name}</span>}
                    style={{padding:0,paddingTop:1,height:'100%'}}
                >
                <Card
                    title={tab.name}
                    bordered={false}
                    style={{height:'100%'}}
                    bodyStyle={{padding:0,minHeight:0,minWidth:0,height:'calc(100vh - 165pt)',overflow:'auto'}}
                    extra={
                        <Button
                            type='outline'
                            icon={<PlusOutlined/>}
                        />
                    }>
                        <ItemList
                            type={tab.type}
                            title={tab.title}
                            description={tab.description}
                        />

                </Card>
            </Tabs.TabPane>
            ))}

        </Tabs>
    );
}
