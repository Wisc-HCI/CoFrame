import React,{useState} from 'react';

import { Tabs, Card, Button,Input,Space } from 'antd';
import { PlusOutlined,SearchOutlined,CloseOutlined  } from '@ant-design/icons';
import useGuiStore from '../../../stores/GuiStore';

export function SearchBox (props){
    const { searchTerm,changeVisibility,clearSearch,visible,onChange,buttonVisible } = props;

   return(
     <Space>
     <Input value ={searchTerm} placeholder ="Search..." onChange={onChange} style={{width:300, left:10,display:visible ? "block":"none"}} />


     <Button ghost size = "small"type="text" icon ={<CloseOutlined/>}style={{position: 'absolute',left: 560,top:20,display:visible ? "block":"none"}}
     onClick={changeVisibility,clearSearch}>

     </Button>
       <Button style={{left:"-5px",display:buttonVisible ?"block":"none"}}type='outline' icon={<SearchOutlined />} onClick={changeVisibility}/>
       <Button
           type='outline'
           icon={<PlusOutlined/>}
       />

     </Space>

);


};
