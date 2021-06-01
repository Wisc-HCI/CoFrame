import React, {useCallback} from 'react';

import { List, Space, Button } from 'antd';
import { DeleteOutlined, EllipsisOutlined, } from '@ant-design/icons';

import useEvdStore from '../../../stores/EvdStore';
import useGuiStore from '../../../stores/GuiStore';


export function RegionItem(props) {

  const { uuid } = props;

  const region = useEvdStore(useCallback(state=>
    state.environment.regions.filter(item=>(item.uuid === uuid))[0]
  ,[uuid]))
  const deleteRegion = useEvdStore(state=>state.deleteRegion);

  const setFocusItem = useGuiStore(state=>state.setFocusItem);

  return (
        <List.Item
          extra={
            <Space align='center'>
              <Button
                onClick={()=>setFocusItem('region',uuid)}
                icon={<EllipsisOutlined/>}
              />
              <Button
                danger
                disabled={!region.canDelete}
                onClick={()=>deleteRegion(uuid)}
                icon={<DeleteOutlined/>}
              />
            </Space>}
          style={{borderRadius:3,backgroundColor:'#1f1f1f',margin:5,padding:10}}
        >
          <List.Item.Meta
            title={region.name}
            description={'Some Description Here'}
          />
        </List.Item>
    );
};

export function RegionList(_) {

  const uuids = useEvdStore(state=>state.environment.regions.map(region=>region.uuid),
    // Custom function to prevent unnecessary re-renders:
    (oldState, newState) => {
      // Only change if the uuids change
      if (newState.environment === undefined || oldState.environment === undefined){
        return false
      } else {
        return oldState.environment.regions.map(region=>region.uuid) === newState.environment.regions.map(region=>region.uuid)
      }
    }
  )

  return (

    <List
      split={false}
      dataSource={uuids}
      renderItem={(uuid)=>(
        <RegionItem uuid={uuid} key={uuid}/>
      )}
    />

  )
}
