import React, {useCallback} from 'react';

import { Drawer, Empty } from 'antd';

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

    if (location) {
        return (
            <Drawer 
                title={'Location: '+location.name}
                visible={focusItem.type === 'location'} 
                onClose={clearFocusItem}
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