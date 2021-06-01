import React, {useCallback} from 'react';

import { Drawer, Empty, Input, Space, Button } from 'antd';

import useGuiStore from '../../stores/GuiStore';

import useEvdStore from '../../stores/EvdStore';

export const LocationDetail = (_) => {

    const {focusItem, clearFocusItem} = useGuiStore(state=>({
        focusItem:state.focusItem,
        clearFocusItem:state.clearFocusItem
    }));

    const {location} = useEvdStore(useCallback(state=>({
        location:state.environment.locations.filter(item=>(item.uuid === focusItem.uuid))[0]
    })
      ,[focusItem]))
      const deleteLocation = useEvdStore(state=>state.deleteLocation);

    if (location) {
        return (
            <Drawer 
                title={<Space><span>Location: </span><Input defaultValue={location.name} disabled={!location.canEdit}/></Space>}
                visible={focusItem.type === 'location'} 
                onClose={clearFocusItem}
                footer={
                    <Button
                        danger
                        block
                        disabled={!location.canDelete}
                        onClick={()=>{
                            clearFocusItem();
                            deleteLocation(focusItem.uuid)
                        }}
                    >
                        Delete
                    </Button>
                }
                width='25%'
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
                width='25%'
            >
                <Empty/>
            </Drawer>
        )

    }
}