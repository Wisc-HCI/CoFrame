import React, {useCallback} from 'react';

import { List } from 'antd';

import { Item } from './Item';

import useEvdStore from '../../../stores/EvdStore';

export function ItemList({type, title, description}) {

    const uuids = useEvdStore(useCallback(state=>Object.keys(state.environment[type+'s']),[type]),
        // Custom function to prevent unnecessary re-renders:
        (oldState, newState) => {
            // Only change if the uuids change
            if (newState.environment === undefined || oldState.environment === undefined){
                return false
            } else {
                return Object.keys(oldState.environment[type+'s']) === Object.keys(newState.environment[type+'s'])
            }
        }
    )
  
    return (
      
      <List
        split={false}
        dataSource={uuids}
        renderItem={(uuid)=>(
          <Item 
            key={uuid}
            uuid={uuid}  
            type={type} 
            title={title} 
            description={description}
          />
        )}
      />
      
    )
  }