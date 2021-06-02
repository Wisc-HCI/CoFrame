import React, {useCallback} from 'react';

import { List, Space, Button,Popconfirm } from 'antd';
import { DeleteOutlined, EllipsisOutlined, } from '@ant-design/icons';

import useEvdStore from '../../../stores/EvdStore';
import useGuiStore from '../../../stores/GuiStore';


export function Item(props) {

    const { type, uuid, title, description } = props;

    const item = useEvdStore(useCallback(state=>
      state.environment[type+'s'][uuid]
    ,[uuid,type]))

    const deleteItem = useEvdStore(state=>state.deleteItem);

    const {focusItem, setFocusItem, primaryColor} = useGuiStore(state=>({
      focusItem:state.focusItem,
      setFocusItem:state.setFocusItem,
      primaryColor:state.primaryColor
    }));
    const [visible, setVisible] = React.useState(false);
    const handleOK = () =>{
      deleteItem(type,uuid);
    }
    const handleCancel = () =>{
       setVisible(false);
    }

    return (
          <List.Item
            extra={
              <Space align='center'>
                <Button
                  onClick={()=>setFocusItem(type,uuid)}
                  icon={<EllipsisOutlined/>}
                />
                <Popconfirm title= "Are you sure you want to delete this item?"
                            onConfirm={handleOK}
                            onCancel ={handleCancel}
                            visible = {visible}
                            placement ="left">
                <Button
                  danger
                  disabled={!item.deleteable}
                  onClick={()=>setVisible(true)}
                  icon={<DeleteOutlined/>}
                />
                  </Popconfirm>
              </Space>}
            style={{
              borderRadius:3,
              backgroundColor:'#1f1f1f',
              margin:5,padding:10,
              boxShadow:focusItem.type === type && focusItem.uuid===uuid ? 'inset 0 0 2.5pt '+primaryColor  : null
            }}
          >
            <List.Item.Meta
              title={title(item)}
              description={description(title)}
            />
          </List.Item>
      );
  };
