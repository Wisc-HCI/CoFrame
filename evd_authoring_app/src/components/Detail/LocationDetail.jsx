import React, {useCallback} from 'react';

import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';

export const LocationDetail = ({uuid}) => {

    const {location} = useEvdStore(useCallback(state=>({
        location:state.data.locations[uuid]
    })
      ,[uuid]))
    // const { deleteItem, setItemProperty } = useEvdStore(state=>({
    //     deleteItem:state.deleteItem,
    //     setItemProperty:state.setItemProperty
    // }));

    console.log(location);
    
    return (<Empty>Custom content for Location</Empty>)
}
