import React, {useCallback} from 'react';

import { Drawer, Empty, Input, Space, Button } from 'antd';

import useGuiStore from '../../stores/GuiStore';

import useEvdStore from '../../stores/EvdStore';

export const MachineDetail = (_) => {

    const {focusItem, clearFocusItem} = useGuiStore(state=>({
        focusItem:state.focusItem,
        clearFocusItem:state.clearFocusItem
    }));

    const {machine} = useEvdStore(useCallback(state=>({
        machine:state.environment.machines.filter(item=>(item.uuid === focusItem.uuid))[0]
    })
      ,[focusItem]))
    const { deleteMachine, setMachineName } = useEvdStore(state=>({
        deleteMachine:state.deleteMachine,
        setMachineName:state.setMachineName
    }));

    if (machine) {
        return (
            <Drawer
                title={
                    <Space>
                        <span>Machine: </span>
                        <Input
                            defaultValue={machine.name}
                            disabled={!machine.canEdit}
                            onChange={e=>setMachineName(focusItem.uuid,e.target.value)}/>
                    </Space>}
                visible={focusItem.type === 'machine'}
                onClose={clearFocusItem}
                getContainer={false}
                style={{ position: 'absolute' }}
                footer={
                    <Button
                        danger
                        block
                        disabled={!machine.canDelete}
                        onClick={()=>{
                            clearFocusItem();
                            deleteMachine(focusItem.uuid)
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
