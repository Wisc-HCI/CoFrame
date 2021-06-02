import React, {useCallback} from 'react';

import { Drawer, Empty, Input, Space, Button } from 'antd';

import useGuiStore from '../../stores/GuiStore';

import useEvdStore from '../../stores/EvdStore';

export const RegionDetail = (_) => {

    const {focusItem, clearFocusItem} = useGuiStore(state=>({
        focusItem:state.focusItem,
        clearFocusItem:state.clearFocusItem
    }));

    const {region} = useEvdStore(useCallback(state=>({
        region:state.environment.regions.filter(item=>(item.uuid === focusItem.uuid))[0]
    })
      ,[focusItem]))
    const { deleteRegion, setRegionName } = useEvdStore(state=>({
        deleteRegion:state.deleteRegion,
        setRegionName:state.setRegionName
    }));

    if (region) {
        return (
            <Drawer
                title={
                    <Space>
                        <span>Region: </span>
                        <Input
                            defaultValue={region.name}
                            disabled={!region.canEdit}
                            onChange={e=>setRegionName(focusItem.uuid,e.target.value)}/>
                    </Space>}
                visible={focusItem.type === 'region'}
                onClose={clearFocusItem}
                getContainer={false}
                style={{ position: 'absolute' }}
                footer={
                    <Button
                        danger
                        block
                        disabled={!region.canDelete}
                        onClick={()=>{
                            clearFocusItem();
                            deleteRegion(focusItem.uuid)
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
