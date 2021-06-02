import React, {useCallback} from 'react';

import { List, Space, Button } from 'antd';
import { DeleteOutlined, EllipsisOutlined, } from '@ant-design/icons';

import useEvdStore from '../../../stores/EvdStore';
import useGuiStore from '../../../stores/GuiStore';


export function MachineItem(props) {

  const { uuid } = props;

  const machine = useEvdStore(useCallback(state=>
    state.environment.machines.filter(item=>(item.uuid === uuid))[0]
  ,[uuid]))
  const deleteMachine = useEvdStore(state=>state.deleteMachine);

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
                onClick={()=>setFocusItem('machine',uuid)}
                icon={<EllipsisOutlined/>}
              />
              <Button
                danger
                disabled={!machine.canDelete}
                onClick={()=>deleteMachine(uuid)}
                icon={<DeleteOutlined/>}
              />
            </Space>}
          style={{
            borderRadius:3,
            backgroundColor:'#1f1f1f',
            margin:5,padding:10,
            boxShadow:focusItem.type === 'machine' && focusItem.uuid===uuid ? 'inset 0 0 2.5pt '+primaryColor  : null
          }}
        >
          <List.Item.Meta
            title={machine.name}
            description={'Some Description Here'}
          />
        </List.Item>
    );
};

export function MachineList(_) {

  const uuids = useEvdStore(state=>state.environment.machines.map(machine=>machine.uuid),
    // Custom function to prevent unnecessary re-renders:
    (oldState, newState) => {
      // Only change if the uuids change
      if (newState.environment === undefined || oldState.environment === undefined){
        return false
      } else {
        return oldState.environment.machines.map(machine=>machine.uuid) === newState.environment.machines.map(machine=>machine.uuid)
      }
    }
  )

  return (

    <List
      split={false}
      dataSource={uuids}
      renderItem={(uuid)=>(
        <MachineItem uuid={uuid} key={uuid}/>
      )}
    />

  )
}
