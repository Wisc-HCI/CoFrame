import React, {useCallback} from 'react';

import { Drawer, Empty, Input, Space, Button,Popconfirm } from 'antd';

import useGuiStore from '../../stores/GuiStore';

import useEvdStore from '../../stores/EvdStore';

export const MachineDetail = (_) => {

    const {focusItem, clearFocusItem} = useGuiStore(state=>({
        focusItem:state.focusItem,
        clearFocusItem:state.clearFocusItem
    }));

    const {machine} = useEvdStore(useCallback(state=>({
        machine:state.environment.machines[focusItem.uuid]
    })
      ,[focusItem]))
      const { deleteItem, setItemProperty } = useEvdStore(state=>({
          deleteItem:state.deleteItem,
          setItemProperty:state.setItemProperty
      }));
     const [visible, setVisible] = React.useState(false);
      const handleOK = () =>{
        clearFocusItem();
        deleteItem('machine',focusItem.uuid);
      }
      const handleCancel = () =>{
         setVisible(false);
      }

    if (machine) {
        return (
            <Drawer
                title={
                    <Space>
                        <span>Machine: </span>
                        <Input
                            defaultValue={machine.name}
                            disabled={!machine.canEdit}
                            onChange={e=>setItemProperty('machine',focusItem.uuid,'name',e.target.value)}/>
                    </Space>}
                visible={focusItem.type === 'machine'}
                onClose={clearFocusItem}
                getContainer={false}
                style={{ position: 'absolute' }}
                footer={
                  <Popconfirm title= "Are you sure you want to delete this machine?"
                              onConfirm={handleOK}
                              onCancel ={handleCancel}
                              visible = {visible}
                              placement ="top">
                    <Button
                        danger
                        block
                        disabled={!machine.deleteable}
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
