import React, {useCallback} from 'react';

import { Drawer, Empty, Input, Space, Button,Popconfirm } from 'antd';

import useGuiStore from '../../stores/GuiStore';

import useEvdStore from '../../stores/EvdStore';

export const RegionDetail = (_) => {

    const {focusItem, clearFocusItem} = useGuiStore(state=>({
        focusItem:state.focusItem,
        clearFocusItem:state.clearFocusItem
    }));

    const {region} = useEvdStore(useCallback(state=>({
        region:state.environment.regions[focusItem.uuid]
    })
      ,[focusItem]))
      const { deleteItem, setItemProperty } = useEvdStore(state=>({
          deleteItem:state.deleteItem,
          setItemProperty:state.setItemProperty
      }));
     const [visible, setVisible] = React.useState(false);
      const handleOK = () =>{
        clearFocusItem();
        deleteItem('region',focusItem.uuid);
      }
      const handleCancel = () =>{
         setVisible(false);
      }

    if (region) {
        return (
            <Drawer
                title={
                    <Space>
                        <span>Region: </span>
                        <Input
                            defaultValue={region.name}
                            disabled={!region.canEdit}
                            onChange={e=>setItemProperty('region',focusItem.uuid,'name',e.target.value)}/>
                    </Space>}
                visible={focusItem.type === 'region'}
                onClose={clearFocusItem}
                getContainer={false}
                style={{ position: 'absolute' }}
                footer={
                  <Popconfirm title= "Are you sure you want to delete this region?"
                              onConfirm={handleOK}
                              onCancel ={handleCancel}
                              visible = {visible}
                              placement ="top">
                    <Button
                        danger
                        block
                        disabled={!region.deleteable}
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
                title={'Region: '}
                visible={focusItem.type === 'region'}
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
