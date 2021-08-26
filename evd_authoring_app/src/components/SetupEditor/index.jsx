import React, { useState } from 'react';
import { Layout, Button, Row } from 'antd';
import { ItemList } from './Tabs/ItemList';
import Icon, { LeftOutlined, RightOutlined } from '@ant-design/icons';
import {ReactComponent as LocationIcon} from '../CustomIcons/Location.svg';
import {ReactComponent as MachineIcon} from '../CustomIcons/Gear.svg';
import {ReactComponent as ThingIcon} from '../CustomIcons/Thing.svg';
import {ReactComponent as WaypointIcon} from '../CustomIcons/Waypoint.svg';
import {SearchBox} from './Tabs/SearchBox.jsx';
import useGuiStore from '../../stores/GuiStore';

export const SetupEditor = () => {

    const [drawerExpanded, setDrawerExpanded] = useState(false);

    const {setupTab, setSetupTab,searchTerm, setSearchTerm, clearSearchTerm} = useGuiStore(state=>({
      setupTab:state.setupTab,
      setSetupTab:state.setSetupTab,
      searchTerm:state.searchTerm,
      setSearchTerm:state.setSearchTerm,
      clearSearchTerm:state.clearSearchTerm
    }));

    const [visible,setVisible] = useState(false);
    const [buttonVisible,setButtonVisible] = useState(true);

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

    const tabs = {
      locations: {
        name:'Locations',
        type:'location',
        title: (item)=> `${item.name}`,
        description: (item)=> `Info about ${item.name}`
      },
      machines: {
        name:'Machines',
        type:'machine',
        title: (item)=> `${item.name}`,
        description: (item)=> `Info about ${item.name}`
      },
      waypoints: {
        name:'Waypoints',
        type:'waypoint',
        title: (item)=> `${item.name}`,
        description: (item)=> `Info about ${item.name}`
      },
      thingTypes: {
        name:'Things',
        type:'thingType',
        title: (item)=> `${item.name}`,
        description: (item)=> `Info about ${item.name}`
      }
    }
    
    console.log(setupTab)
    const toggle = () => setDrawerExpanded(!drawerExpanded);
    
    return (
        <Layout style={{ flex: 1, fontSize: 20 }}>
          <Layout.Sider collapsible collapsed={!drawerExpanded} trigger={null} style={{ align: 'left', display: 'flex', flexDirection: 'column', padding: 5 }}>
            <Button key='toggle' type='ghost' block icon={drawerExpanded ? <LeftOutlined /> : <RightOutlined />} onClick={toggle} style={{ marginBottom: 5 }} />
            <Button key='machines' type={setupTab==='machines'?'primary':'text'} block icon={<Icon component={MachineIcon}/>} style={{ marginBottom: 5, alignItems: 'left' }} onClick={()=>setSetupTab('machines')}>
                {drawerExpanded && 'Machines'}
            </Button>
            <Button key='locations' type={setupTab==='locations'?'primary':'text'} block icon={<Icon component={LocationIcon}/>} style={{ marginBottom: 5, alignItems: 'left' }} onClick={()=>setSetupTab('locations')}>
                {drawerExpanded && 'Locations'}
            </Button>
            <Button key='waypoints' type={setupTab==='waypoints'?'primary':'text'} block icon={<Icon component={WaypointIcon}/>} style={{ marginBottom: 5, alignItems: 'left' }} onClick={()=>setSetupTab('waypoints')}>
                {drawerExpanded && 'Waypoints'}
            </Button>
            <Button key='things' type={setupTab==='thingTypes'?'primary':'text'} block icon={<Icon component={ThingIcon}/>} style={{ marginBottom: 5, alignItems: 'left' }} onClick={()=>setSetupTab('thingTypes')}>
                {drawerExpanded && 'Things'}
            </Button>
          </Layout.Sider>
          <Layout.Content style={{ height: 'calc(100vh - 115pt)'}}>
            <Row align='middle' justify='space-between' style={{padding:5,marginLeft:5}}>
              <span style={{color:'white',padding:3}}>{tabs[setupTab].name}</span>
              <SearchBox searchTerm={searchTerm} changeVisibility = {changeVisibility} clearSearch={clearSearch} onChange = {onChange} visible={visible} buttonVisible = {buttonVisible} />
            </Row>
            <div style={{ height: 'calc(100vh - 147pt)', overflow: 'scroll' }}>
              <ItemList
                  type={tabs[setupTab].type}
                  title={tabs[setupTab].title}
                  description={tabs[setupTab].description}
                  searchTerm={searchTerm}
              />
            </div>
            
            
            
          </Layout.Content>
        </Layout>
    )
}

// import React,{useState} from 'react';

// import { Tabs, Card } from 'antd';

// import { ItemList } from './Tabs/ItemList';
// // import { WaypointList } from './Tabs/Waypoints';
// // import { RegionList } from './Tabs/Regions';
// // import { MachineList } from './Tabs/Machines';
// // import { ThingList } from './Tabs/Things';
// // import { ThingTypeList } from './Tabs/ThingTypes';

// import useGuiStore from '../../stores/GuiStore';


// export function SetupEditor(_) {

//     const {primaryColor, setupTab, setSetupTab} = useGuiStore(state=>({
//         primaryColor:state.primaryColor,
//         setupTab:state.setupTab,
//         setSetupTab:state.setSetupTab
//     }));


//     const tabs = [
//         {
//             key:'locations',
//             name:'Locations',
//             type:'location',
//             title: (item)=> `${item.name}`,
//             description: (item)=> `Info about ${item.name}`
//         },
//         {
//             key:'machines',
//             name:'Machines',
//             type:'machine',
//             title: (item)=> `${item.name}`,
//             description: (item)=> `Info about ${item.name}`
//         },
//         {
//             key:'waypoints',
//             name:'Waypoints',
//             type:'waypoint',
//             title: (item)=> `${item.name}`,
//             description: (item)=> `Info about ${item.name}`
//         },
//         {
//             key:'thingTypes',
//             name:'Things',
//             type:'thingType',
//             title: (item)=> `${item.name}`,
//             description: (item)=> `Info about ${item.name}`
//         }
//     ]
//     //const { TextArea } = Input;
//     const [visible,setVisible] = useState(false);
//     const [buttonVisible,setButtonVisible] = useState(true);
//     const {searchTerm, setSearchTerm, clearSearchTerm} = useGuiStore(state=>({
//       searchTerm:state.searchTerm,
//       setSearchTerm:state.setSearchTerm,
//       clearSearchTerm:state.clearSearchTerm
//     }));

//     const changeVisibility = () => {
//       setVisible(!visible);
//       setButtonVisible(!buttonVisible);
//     }
//     const clearSearch = () =>{
//       setVisible(!visible);
//       setButtonVisible(!buttonVisible);
//       clearSearchTerm();
//     }

//     const onChange = (event) => {
//       setSearchTerm(event.target.value);
//     };



//     return (

//         <Tabs
//             tabPosition='left'
//             style={{display:'flex',flex:1}}
//             defaultActiveKey={setupTab}
//             onChange={setSetupTab}

//         >
//         {tabs.map((tab)=>{
//           return (

//             <Tabs.TabPane
//                 key={tab.key}
//                 tab={<span style={tab.key === setupTab ? { color: primaryColor } : {}}>{tab.name}</span>}

//                 style={{padding:0,paddingTop:1,height:'100%'}}
//             >
//             <Card
//                 title={tab.name}
//                 bordered={false}
//                 style={{height:'100%'}}
//                 bodyStyle={{padding:0,minHeight:0,minWidth:0,height:'calc(100vh - 165pt)',overflow:'auto'}}
//                 extra={
//                   <SearchBox searchTerm={searchTerm} changeVisibility = {changeVisibility} clearSearch={clearSearch} onChange = {onChange} visible={visible} buttonVisible = {buttonVisible} />
//                 }>
//                     <ItemList
//                         type={tab.type}
//                         title={tab.title}
//                         description={tab.description}
//                         searchTerm={searchTerm}
//                     />

//             </Card>
//         </Tabs.TabPane>



//           );


//         })}





//         </Tabs>
//     );
// }


