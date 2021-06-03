import React, {useCallback} from 'react';

import { Drawer, Empty, Input, Space, Button,Popconfirm } from 'antd';

import useGuiStore from '../../stores/GuiStore';

import useEvdStore from '../../stores/EvdStore';

export const ThingDetail = (_) => {

    const {focusItem, clearFocusItem} = useGuiStore(state=>({
        focusItem:state.focusItem,
        clearFocusItem:state.clearFocusItem
    }));

    const {thing} = useEvdStore(useCallback(state=>({
        thing:state.environment.things[focusItem.uuid]
    })
      ,[focusItem]))
      const { deleteItem, setItemProperty } = useEvdStore(state=>({
          deleteItem:state.deleteItem,
          setItemProperty:state.setItemProperty
      }));
     const [visible, setVisible] = React.useState(false);
      const handleOK = () =>{
        clearFocusItem();
        deleteItem('thing',focusItem.uuid);
      }
      const handleCancel = () =>{
         setVisible(false);
      }

    if (thing) {
        return (
            <Drawer
                title={
                    <Space>
                        <span>Thing: </span>
                        <Input
                            defaultValue={thing.name}
                            disabled={!thing.editable}
                            onChange={e=>setItemProperty('thing',focusItem.uuid,'name',e.target.value)}/>
                    </Space>}
                visible={focusItem.type === 'thing'}
                onClose={clearFocusItem}
                getContainer={false}
                style={{ position: 'absolute' }}
                footer={
                  <Popconfirm title= "Are you sure you want to delete this thing?"
                              onConfirm={handleOK}
                              onCancel ={handleCancel}
                              visible = {visible}
                              placement ="top">
                    <Button
                        danger
                        block
                        disabled={!thing.deleteable}
                        onClick={()=>setVisible(true)}
                    >
                        Delete
                    </Button>
                    </Popconfirm>
                }
                width='50%'
            >
                <Empty/>
            </Drawer>
        )
    } else {
        return  (
            <Drawer
                title={'Thing: '}
                visible={focusItem.type === 'thing'}
                onClose={clearFocusItem}
                getContainer={false}
                style={{ position: 'absolute' }}
                width='50%'
            >
                <Empty/>
            </Drawer>
        )

    }
}
