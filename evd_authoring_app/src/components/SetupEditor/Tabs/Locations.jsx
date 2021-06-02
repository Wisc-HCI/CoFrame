import React, {useCallback} from 'react';

import { List, Space, Button,Popconfirm } from 'antd';
import { DeleteOutlined, EllipsisOutlined, } from '@ant-design/icons';

import useEvdStore from '../../../stores/EvdStore';
import useGuiStore from '../../../stores/GuiStore';


export function LocationItem(props) {

  const { uuid } = props;

  const location = useEvdStore(useCallback(state=>
    state.environment.locations.filter(item=>(item.uuid === uuid))[0]
  ,[uuid]))
  const deleteLocation = useEvdStore(state=>state.deleteLocation);

  const {focusItem, setFocusItem, primaryColor} = useGuiStore(state=>({
    focusItem:state.focusItem,
    setFocusItem:state.setFocusItem,
    primaryColor:state.primaryColor
  }));
  const [visible, setVisible] = React.useState(false);
  const handleOK = () =>{
    deleteLocation(uuid);
  }
  const handleCancel = () =>{
     setVisible(false);
  }


  return (
        <List.Item
          extra={
            <Space align='center'>
              <Button
                onClick={()=>setFocusItem('location',uuid)}
                icon={<EllipsisOutlined/>}
              />
              <Popconfirm title= "Are you sure you want to delete this location?"
                          onConfirm={handleOK}
                          onCancel ={handleCancel}
                          visible = {visible}
                          placement ="left">
              <Button
                danger
                disabled={!location.canDelete}
                onClick={()=>setVisible(true)}
                icon={<DeleteOutlined/>}
              />
              </Popconfirm>
            </Space>}
          style={{
            borderRadius:3,
            backgroundColor:'#1f1f1f',
            margin:5,padding:10,
            boxShadow:focusItem.type === 'location' && focusItem.uuid===uuid ? 'inset 0 0 2.5pt '+primaryColor  : null
          }}
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
