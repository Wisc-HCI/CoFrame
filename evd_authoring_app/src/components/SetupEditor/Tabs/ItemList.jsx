import React, {useCallback} from 'react';

import { List } from 'antd';

import { Item } from './Item';

import useEvdStore from '../../../stores/EvdStore';

export function ItemList({type, title, description, searchTerm}) {

  const uuids = useEvdStore(useCallback(state=>Object.keys(state.environment[type+'s'])
        .filter(uuid=>{
          if (searchTerm === '') {
            return true
          } else {
            return state.environment[type+'s'][uuid].name.toLowerCase().includes(searchTerm.toLowerCase())
          }
        }),[type,searchTerm])
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
