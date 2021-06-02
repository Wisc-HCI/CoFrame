import React, {useCallback} from 'react';

import { Drawer, Empty, Input, Space, Button } from 'antd';

import useGuiStore from '../../stores/GuiStore';

import useEvdStore from '../../stores/EvdStore';

export const WaypointDetail = (_) => {

    const {focusItem, clearFocusItem} = useGuiStore(state=>({
        focusItem:state.focusItem,
        clearFocusItem:state.clearFocusItem
    }));

    const {waypoint} = useEvdStore(useCallback(state=>({
        waypoint:state.environment.waypoints.filter(item=>(item.uuid === focusItem.uuid))[0]
    })
      ,[focusItem]))
    const { deleteWaypoint, setWaypointName } = useEvdStore(state=>({
        deleteWaypoint:state.deleteWaypoint,
        setWaypointName:state.setWaypointName
    }));

    if (waypoint) {
        return (
            <Drawer
                title={
                    <Space>
                        <span>Waypoint: </span>
                        <Input
                            defaultValue={waypoint.name}
                            disabled={!waypoint.canEdit}
                            onChange={e=>setWaypointName(focusItem.uuid,e.target.value)}/>
                    </Space>}
                visible={focusItem.type === 'waypoint'}
                onClose={clearFocusItem}
                getContainer={false}
                style={{ position: 'absolute' }}
                footer={
                    <Button
                        danger
                        block
                        disabled={!waypoint.canDelete}
                        onClick={()=>{
                            clearFocusItem();
                            deleteWaypoint(focusItem.uuid)
                        }}
                    >
                        Delete
                    </Button>
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
