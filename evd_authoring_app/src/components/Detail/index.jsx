import React, {useCallback} from 'react';

import { Drawer, Input, Empty, Space, Button, Popover } from 'antd';

import useGuiStore from '../../stores/GuiStore';

import { LocationDetail } from './LocationDetail';
import { MachineDetail } from './MachineDetail';
import { ThingDetail } from './ThingDetail';
import { WaypointDetail } from './WaypointDetail';
import { DeleteOutlined, EllipsisOutlined, } from '@ant-design/icons';

import useEvdStore from '../../stores/EvdStore';

export const Detail = (_) => {

    const {focusItem, clearFocusItem} = useGuiStore(state=>({
        focusItem:state.focusItem,
        clearFocusItem:state.clearFocusItem
    }));

    const {item} = useEvdStore(useCallback(state=>({
        item:focusItem.type ? state.environment[focusItem.type+'s'][focusItem.uuid] : null
    }),[focusItem]))

    const { deleteItem, setItemProperty } = useEvdStore(state=>({
        deleteItem:state.deleteItem,
        setItemProperty:state.setItemProperty
    }));

    const [popoverVisible, setPopoverVisible] = React.useState(false);

    const handleOK = () =>{
        clearFocusItem();
        deleteItem(focusItem.type,focusItem.uuid);
    }

    const content =(
      <Button
        danger
        onClick={handleOK}
        icon={<DeleteOutlined/>}
        style = {{width:"300px"}}
      >
      Delete
      </Button>

    )

    if (item) {
        return (
            <Drawer
                title={
                    <Space>
                        <span style={{textTransform:'capitalize'}}>{focusItem.type} </span>
                        <Input
                            defaultValue={item.name}
                            disabled={!item.editable}
                            onChange={e=>setItemProperty(focusItem.type,focusItem.uuid,'name',e.target.value)}/>
                    </Space>}
                visible={focusItem.uuid !== null && focusItem.type !== null}
                onClose={clearFocusItem}
                getContainer={false}
                style={{ position: 'absolute' }}
                footer={
                  <Popover title={`Are you sure you want to delete this ${focusItem.type}?`}
                           trigger = "click"
                           placement ="top"
                           content = {content}>
                    <Button
                        danger
                        block
                        disabled={!item.deleteable}
                        onClick={()=>setPopoverVisible(true)}
                        icon={<DeleteOutlined/>}
                    >
                        Delete
                    </Button>
                    </Popover>
                }
                width='50%'
            >
                {focusItem.type === 'location' && (
                    <LocationDetail uuid={focusItem.uuid}/>
                )}
                {focusItem.type === 'machine' && (
                    <MachineDetail uuid={focusItem.uuid}/>
                )}
                {focusItem.type === 'waypoint' && (
                    <WaypointDetail uuid={focusItem.uuid}/>
                )}
                {focusItem.type === 'thingType' && (
                    <ThingDetail uuid={focusItem.uuid}/>
                )}
                {focusItem.type === null && (
                    <Empty/>
                )}
            </Drawer>
        )
    } else {
        return  (
            <Drawer
                title={<span style={{textTransform:'capitalize'}}>{focusItem.type} </span>}
                visible={focusItem.uuid !== null && focusItem.type !== null}
                onClose={clearFocusItem}
                getContainer={false}
                style={{ position: 'absolute' }}
                width='50%'
            >
            </Drawer>
        )

    }
}
