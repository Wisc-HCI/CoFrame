import React from 'react';

import { Tabs, Card, Button } from 'antd';
import { PlusOutlined } from '@ant-design/icons';

import { LocationList } from './Tabs/Locations';
import { WaypointList } from './Tabs/Waypoints';
import { RegionList } from './Tabs/Regions';
import { MachineList } from './Tabs/Machines';
// import { ThingList } from './Tabs/Things';
// import { ThingTypeList } from './Tabs/ThingTypes';


import useGuiStore from '../../stores/GuiStore';
import frameStyles from '../../frameStyles';

// import './index.css'


export function SetupEditor(_) {

    const {frame, setupTab, setSetupTab} = useGuiStore(state=>({
        frame:state.frame,
        setupTab:state.setupTab,
        setSetupTab:state.setSetupTab
    }));

    const tabs = [
        {
            key:'locations',
            name:'Locations',
            content: <LocationList/>
        },
        {
            key:'machines',
            name:'Machines',
            content: <MachineList/>
        },
        // {
        //     key:'regions',
        //     name:'Regions',
        //     content: <RegionList/>

        // },
        // {
        //     key:'waypoints',
        //     name:'Waypoints',
        //     content: <WaypointList/>

        // }
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
                    tab={<span style={tab.key === setupTab ? { color: frameStyles.colors[frame] } : {}}>{tab.name}</span>}
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
                        {tab.content}

                </Card>
            </Tabs.TabPane>
            ))}

        </Tabs>
    );
}
