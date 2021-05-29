import React, {useCallback} from 'react';

import { List, Space, Button } from 'antd';
import { DeleteOutlined, EditOutlined, } from '@ant-design/icons';

import useEvdStore from '../../../stores/EvdStore';


export function LocationItem(props) {

  const { uuid } = props;

  const location = useEvdStore(useCallback(state=>
    state.environment.locations.filter(item=>(item.uuid === uuid))[0]
  ,[uuid]))

  return (
        <List.Item
          extra={
            <Space align='center'>
              <Button 
                disabled={!location.canEdit}
                icon={<EditOutlined/>}
              />
              <Button 
                danger
                disabled={!location.canDelete}
                icon={<DeleteOutlined/>}
              />
            </Space>}
          style={{borderRadius:3,backgroundColor:'#1f1f1f',margin:5,padding:10}}
        >
          <List.Item.Meta
            title={location.name}
            description={'Some Description Here'}
          />
        </List.Item>
    );
};

export function LocationList(_) {
  
  const uuids = useEvdStore(state=>state.environment.locations.map(location=>location.uuid),
    // Custom function to prevent unnecessary re-renders:
    (oldState, newState) => {
      // Only change if the uuids change
      if (newState.environment === undefined || oldState.environment === undefined){
        return false
      } else {
        return oldState.environment.locations.map(location=>location.uuid) === newState.environment.locations.map(location=>location.uuid)
      }
    }
  )

  return (
    
    <List
      split={false}
      dataSource={uuids}
      renderItem={(uuid)=>(
        <LocationItem uuid={uuid} key={uuid}/>
      )}
    />
    
  )
}
