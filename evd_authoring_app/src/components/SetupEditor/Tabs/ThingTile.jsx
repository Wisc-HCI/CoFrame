import React, {useCallback} from 'react';

import { List, Space, Button } from 'antd';
import { DeleteOutlined, EllipsisOutlined, } from '@ant-design/icons';

import useEvdStore from '../../../stores/EvdStore';
import useGuiStore from '../../../stores/GuiStore';


export function ThingTileItem(props) {

  const { uuid } = props;

  const thingTile = useEvdStore(useCallback(state=>
    state.environment.things.filter(item=>(item.uuid === uuid))[0]
  ,[uuid]))
  const deleteThing = useEvdStore(state=>state.deleteThing);

  const {focusItem, setFocusItem, primaryColor} = useGuiStore(state=>({
    focusItem:state.focusItem,
    setFocusItem:state.setFocusItem,
    primaryColor:state.primaryColor
  }));

  return (
        <List.Item
          extra={
            <Space align='center'>
              <Button
                onClick={()=>setFocusItem('thing',uuid)}
                icon={<EllipsisOutlined/>}
              />
              <Button
                danger
                disabled={!thingTile.canDelete}
                onClick={()=>deleteThing(uuid)}
                icon={<DeleteOutlined/>}
              />
            </Space>}
          style={{
            borderRadius:3,
            backgroundColor:'#1f1f1f',
            margin:5,padding:10,
            boxShadow:focusItem.type === 'thing' && focusItem.uuid===uuid ? 'inset 0 0 2.5pt '+primaryColor  : null
          }}
        >
          <List.Item.Meta
            title={thingTile.name}
            description={'Some Description Here'}
          />
        </List.Item>
    );
};

export function ThingTileList(_) {

  const uuids = useEvdStore(state=>state.environment.things.map(thing=>thing.uuid),
    // Custom function to prevent unnecessary re-renders:
    (oldState, newState) => {
      // Only change if the uuids change
      if (newState.environment === undefined || oldState.environment === undefined){
        return false
      } else {
        return oldState.environment.things.map(thing=>thing.uuid) === newState.environment.things.map(thing=>thing.uuid)
      }
    }
  )

  return (

    <List
      split={false}
      dataSource={uuids}
      renderItem={(uuid)=>(
        <ThingTileItem uuid={uuid} key={uuid}/>
      )}
    />

  )
}
