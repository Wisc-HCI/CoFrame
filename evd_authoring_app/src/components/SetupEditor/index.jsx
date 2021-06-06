import React,{useState} from 'react';

import { Tabs, Card, Button,Input,Space } from 'antd';
import { PlusOutlined,SearchOutlined,CloseOutlined  } from '@ant-design/icons';

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
    const [visible,setVisible] = useState(false);
    const [ButtonVisible,setButtonVisible] = useState(true);
    const [searchTerm,setSearchTerm] = useState("");

    const changeVisibility = () => {
      setVisible(!visible);
      setButtonVisible(!ButtonVisible);
      setSearchTerm("");



    }
    const onChange = (event) => {
      setSearchTerm(event.target.value);
    };

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
    const { TextArea } = Input;



    return (

        <Tabs
            tabPosition='left'
            style={{display:'flex',flex:1}}
            defaultActiveKey={setupTab}
            onChange={setSetupTab}

        >
        {tabs.filter((tab) => {
          if (searchTerm ==""){
            return tab;
          } else if (tab.name.toLowerCase().includes(searchTerm.toLowerCase())){

            return tab;
          }

        }).map((tab)=>{
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
                  <Space>
                  <Input value ={searchTerm} placeholder ="Search..." onChange={onChange} style={{width:300, left:10,display:visible ? "block":"none"}} />


                  <Button ghost size = "small"type="text" icon ={<CloseOutlined/>}style={{position: 'absolute',left: 560,top:20,display:visible ? "block":"none"}}
                  onClick={changeVisibility}>

                  </Button>
                    <Button style={{left:"-5px",display:ButtonVisible ?"block":"none"}}type='outline' icon={<SearchOutlined />} onClick={changeVisibility}/>
                    <Button
                        type='outline'
                        icon={<PlusOutlined/>}
                    />

                  </Space>

                }>
                    <ItemList
                        type={tab.type}
                        title={tab.title}
                        description={tab.description}
                    />

            </Card>
        </Tabs.TabPane>



          );


        })}





        </Tabs>
    );
}
