import React, {useCallback} from 'react';

import { List, Space, Button } from 'antd';
import { DeleteOutlined, EllipsisOutlined, } from '@ant-design/icons';

import useEvdStore from '../../../stores/EvdStore';
import useGuiStore from '../../../stores/GuiStore';


export function ThingTypesItem(props) {

  const { uuid } = props;

  const thingtypes = useEvdStore(useCallback(state=>
    state.environment.thingTypes.filter(item=>(item.uuid === uuid))[0]
  ,[uuid]))
  const deleteThingTypes = useEvdStore(state=>state.deleteThingTypes);

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
                onClick={()=>setFocusItem('thingTypes',uuid)}
                icon={<EllipsisOutlined/>}
              />
              <Button
                danger
                disabled={!thingtypes.canDelete}
                onClick={()=>deleteThingTypes(uuid)}
                icon={<DeleteOutlined/>}
              />
            </Space>}
          style={{
            borderRadius:3,
            backgroundColor:'#1f1f1f',
            margin:5,padding:10,
            boxShadow:focusItem.type === 'thingTypes' && focusItem.uuid===uuid ? 'inset 0 0 2.5pt '+primaryColor  : null
          }}
        >
          <List.Item.Meta
            title={thingtypes.name}
            description={'Some Description Here'}
          />
        </List.Item>
    );
};

export function ThingTypesList(_) {

  const uuids = useEvdStore(state=>state.environment.thingTypes.map(thingtypes=>thingtypes.uuid),
    // Custom function to prevent unnecessary re-renders:
    (oldState, newState) => {
      // Only change if the uuids change
      if (newState.environment === undefined || oldState.environment === undefined){
        return false
      } else {
        return oldState.environment.thingTypes.map(thingtypes=>thingtypes.uuid) === newState.environment.thingTypes.map(thingtypes=>thingtypes.uuid)
      }
    }
  )

  return (

    <List
      split={false}
      dataSource={uuids}
      renderItem={(uuid)=>(
        <ThingTypesItem uuid={uuid} key={uuid}/>
      )}
    />

  )
}
