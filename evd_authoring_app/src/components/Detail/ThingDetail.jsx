import React, {useCallback} from 'react';

import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';

export const ThingDetail = ({uuid}) => {

    const {thingType} = useEvdStore(useCallback(state=>({
        thingType:state.environment.thingTypes[uuid]
    })
      ,[uuid]))
    const { deleteItem, setItemProperty } = useEvdStore(state=>({
        deleteItem:state.deleteItem,
        setItemProperty:state.setItemProperty
    }));

    console.log(thingType);
    
    return (<Empty>Custom content for Thing Type</Empty>)
}
