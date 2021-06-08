import React,{useState} from 'react';

import { Tabs, Card } from 'antd';

import { ItemList } from './Tabs/ItemList';
// import { WaypointList } from './Tabs/Waypoints';
// import { RegionList } from './Tabs/Regions';
// import { MachineList } from './Tabs/Machines';
// import { ThingList } from './Tabs/Things';
// import { ThingTypeList } from './Tabs/ThingTypes';

import useGuiStore from '../../stores/GuiStore';
import {SearchBox} from './Tabs/SearchBox.jsx';

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
            type:'thing',
            title: (item)=> `${item.name}`,
            description: (item)=> `Info about ${item.name}`
        },
        {
            key:'thingTypes',
            name:'Thing Types',
            type:'thingType',
            title: (item)=> `${item.name}`,
            description: (item)=> `Info about ${item.name}`
        }
    ]
    //const { TextArea } = Input;
    const [visible,setVisible] = useState(false);
    const [buttonVisible,setButtonVisible] = useState(true);
    const {searchTerm, setSearchTerm, clearSearchTerm} = useGuiStore(state=>({
      searchTerm:state.searchTerm,
      setSearchTerm:state.setSearchTerm,
      clearSearchTerm:state.clearSearchTerm
    }));

    const changeVisibility = () => {
      setVisible(!visible);
      setButtonVisible(!buttonVisible);
    }
    const clearSearch = () =>{
      setVisible(!visible);
      setButtonVisible(!buttonVisible);
      clearSearchTerm();
    }

    const onChange = (event) => {
      setSearchTerm(event.target.value);
    };



    return (

        <Tabs
            tabPosition='left'
            style={{display:'flex',flex:1}}
            defaultActiveKey={setupTab}
            onChange={setSetupTab}

        >
        {tabs.map((tab)=>{
          return (

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
                  <SearchBox searchTerm={searchTerm} changeVisibility = {changeVisibility} clearSearch={clearSearch} onChange = {onChange} visible={visible} buttonVisible = {buttonVisible} />
                }>
                    <ItemList
                        type={tab.type}
                        title={tab.title}
                        description={tab.description}
                        searchTerm={searchTerm}
                    />

            </Card>
        </Tabs.TabPane>



          );


        })}





        </Tabs>
    );
}
