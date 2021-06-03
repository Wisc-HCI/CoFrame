import React, {useCallback} from 'react';

import { Drawer, Empty, Input, Space, Button,Popconfirm } from 'antd';

import useGuiStore from '../../stores/GuiStore';

import useEvdStore from '../../stores/EvdStore';

export const LocationDetail = (_) => {

    const {focusItem, clearFocusItem} = useGuiStore(state=>({
        focusItem:state.focusItem,
        clearFocusItem:state.clearFocusItem
    }));

    const {location} = useEvdStore(useCallback(state=>({
        location:state.environment.locations[focusItem.uuid]
    })
      ,[focusItem]))
    const { deleteItem, setItemProperty } = useEvdStore(state=>({
        deleteItem:state.deleteItem,
        setItemProperty:state.setItemProperty
    }));
    const [visible, setVisible] = React.useState(false);
    const handleOK = () =>{
      clearFocusItem();
      deleteItem('location',focusItem.uuid);
    }
    const handleCancel = () =>{
       setVisible(false);
    }

    if (location) {
        return (
            <Drawer
                title={
                    <Space>
                        <span>Location: </span>
                        <Input
                            defaultValue={location.name}
                            disabled={!location.canEdit}
                            onChange={e=>setItemProperty('location',focusItem.uuid,'name',e.target.value)}/>
                    </Space>}
                visible={focusItem.type === 'location'}
                onClose={clearFocusItem}
                getContainer={false}
                style={{ position: 'absolute' }}
                footer={
                  <Popconfirm title= "Are you sure you want to delete this location?"
                              onConfirm={handleOK}
                              onCancel ={handleCancel}
                              visible = {visible}
                              placement ="top">
                    <Button
                        danger
                        block
                        disabled={!location.deleteable}
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
                title={'Location: '}
                visible={focusItem.type === 'location'}
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
