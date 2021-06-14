import React, {useCallback} from 'react';

import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';

export const WaypointDetail = ({uuid}) => {

    const {waypoint} = useEvdStore(useCallback(state=>({
        waypoint:state.data.waypoints[uuid]
    })
      ,[uuid]))
    // const { deleteItem, setItemProperty } = useEvdStore(state=>({
    //     deleteItem:state.deleteItem,
    //     setItemProperty:state.setItemProperty
    // }));

    console.log(waypoint);
    
    return (<Empty>Custom content for Waypoint</Empty>)
}
