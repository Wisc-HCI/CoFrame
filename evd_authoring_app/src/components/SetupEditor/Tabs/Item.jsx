import React, {useCallback,useRef,useEffect} from 'react';

import { List, Space, Button,Popconfirm,Popover,Alert } from 'antd';
import { DeleteOutlined, EllipsisOutlined, } from '@ant-design/icons';
import 'antd/dist/antd.css';

import useEvdStore from '../../../stores/EvdStore';
import useGuiStore from '../../../stores/GuiStore';


export function Item(props) {

    const { type, uuid, title, description} = props;

    const item = useEvdStore(useCallback(state=>
      state.environment[type+'s'][uuid]
    ,[uuid,type]))

    const deleteItem = useEvdStore(state=>state.deleteItem);

    const {focusItem, setFocusItem, primaryColor} = useGuiStore(state=>({
      focusItem:state.focusItem,
      setFocusItem:state.setFocusItem,
      primaryColor:state.primaryColor
    }));


    const content =(
      <Button
        danger
        onClick={()=>deleteItem(type,uuid)}
        icon={<DeleteOutlined/>}
        style = {{width:"300px"}}
      >
      Delete
      </Button>

    )


      return (
            <List.Item
              extra={
                <Space align='center'>
                  <Button
                    onClick={()=>setFocusItem(type,uuid)}
                    icon={<EllipsisOutlined/>}
                  />


                  <div>
                  {item.deleteable ? (
                      <Popover title= "Are you sure you want to delete this item?"
                               trigger = "click"
                               placement ="left"
                               content = {content}>
                                  <Button
                                    danger
                                    disabled={!item.deleteable}
                                    icon={<DeleteOutlined/>}
                                  />

                        </Popover>
                    ):(
                      <Button
                        danger
                        disabled={!item.deleteable}
                        icon={<DeleteOutlined/>}
                      />

                    )}


                  </div>


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
