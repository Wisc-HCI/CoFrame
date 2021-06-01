import React, {useCallback} from 'react';

import { List, Space, Button } from 'antd';
import { DeleteOutlined, EllipsisOutlined, } from '@ant-design/icons';

import useEvdStore from '../../../stores/EvdStore';
import useGuiStore from '../../../stores/GuiStore';


export function WaypointItem(props) {

  const { uuid } = props;

  const waypoint = useEvdStore(useCallback(state=>
    state.environment.waypoints.filter(item=>(item.uuid === uuid))[0]
  ,[uuid]))
  const deleteWaypoint = useEvdStore(state=>state.deleteWaypoint);

  const setFocusItem = useGuiStore(state=>state.setFocusItem);

  return (
        <List.Item
          extra={
            <Space align='center'>
              <Button
                onClick={()=>setFocusItem('waypoint',uuid)}
                icon={<EllipsisOutlined/>}
              />
              <Button
                danger
                disabled={!waypoint.canDelete}
                onClick={()=>deleteWaypoint(uuid)}
                icon={<DeleteOutlined/>}
              />
            </Space>}
          style={{borderRadius:3,backgroundColor:'#1f1f1f',margin:5,padding:10}}
        >
          <List.Item.Meta
            title={waypoint.name}
            description={'Some Description Here'}
          />
        </List.Item>
    );
};

export function WaypointList(_) {

  const uuids = useEvdStore(state=>state.environment.waypoints.map(waypoint=>waypoint.uuid),
    // Custom function to prevent unnecessary re-renders:
    (oldState, newState) => {
      // Only change if the uuids change
      if (newState.environment === undefined || oldState.environment === undefined){
        return false
      } else {
        return oldState.environment.waypoints.map(waypoint=>waypoint.uuid) === newState.environment.waypoints.map(waypoint=>waypoint.uuid)
      }
    }
  )

  return (

    <List
      split={false}
      dataSource={uuids}
      renderItem={(uuid)=>(
        <WaypointItem uuid={uuid} key={uuid}/>
      )}
    />

  )
}
