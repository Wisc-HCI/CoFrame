import React, {useCallback} from 'react';

import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';

export const MachineDetail = ({uuid}) => {

    const {machine} = useEvdStore(useCallback(state=>({
        machine:state.environment.machines[uuid]
    })
      ,[uuid]))
    const { deleteItem, setItemProperty } = useEvdStore(state=>({
        deleteItem:state.deleteItem,
        setItemProperty:state.setItemProperty
    }));

    console.log(machine);
    
    return (<Empty>Custom content for Machine</Empty>)
}
