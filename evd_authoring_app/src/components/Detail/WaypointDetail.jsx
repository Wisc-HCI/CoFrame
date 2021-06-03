import React, {useCallback} from 'react';

import { Drawer, Empty, Input, Space, Button ,Popconfirm} from 'antd';

import useGuiStore from '../../stores/GuiStore';

import useEvdStore from '../../stores/EvdStore';

export const WaypointDetail = (_) => {

    const {focusItem, clearFocusItem} = useGuiStore(state=>({
        focusItem:state.focusItem,
        clearFocusItem:state.clearFocusItem
    }));

    const {waypoint} = useEvdStore(useCallback(state=>({
        waypoint:state.environment.waypoints[focusItem.uuid]
    })
      ,[focusItem]))
      const { deleteItem, setItemProperty } = useEvdStore(state=>({
          deleteItem:state.deleteItem,
          setItemProperty:state.setItemProperty
      }));
    const [visible, setVisible] = React.useState(false);
     const handleOK = () =>{
       clearFocusItem();
       deleteItem('waypoint',focusItem.uuid);
     }
     const handleCancel = () =>{
        setVisible(false);
     }

    if (waypoint) {
        return (
            <Drawer
                title={
                    <Space>
                        <span>Waypoint: </span>
                        <Input
                            defaultValue={waypoint.name}
                            disabled={!waypoint.canEdit}
                            onChange={e=>setItemProperty('waypoint',focusItem.uuid,'name',e.target.value)}/>
                    </Space>}
                visible={focusItem.type === 'waypoint'}
                onClose={clearFocusItem}
                getContainer={false}
                style={{ position: 'absolute' }}
                footer={
                  <Popconfirm title= "Are you sure you want to delete this waypoint?"
                              onConfirm={handleOK}
                              
                              onCancel ={handleCancel}
                              visible = {visible}
                              placement ="top">
                    <Button
                        danger
                        block
                        disabled={!waypoint.deleteable}
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
                title={'Waypoint: '}
                visible={focusItem.type === 'waypoint'}
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
