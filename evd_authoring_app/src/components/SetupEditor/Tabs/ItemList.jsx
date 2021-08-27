import React, { useCallback } from 'react';

import { List } from 'antd';

import { Item } from './Item';

import useStore from '../../../stores/Store';

export function ItemList({ type, title, description, searchTerm }) {

  const uuids = useStore(useCallback(state => Object.keys(state.data[type + 's'])
    .filter(uuid => {
      if (searchTerm === '') {
        return true
      } else {
        return state.data[type + 's'][uuid].name.toLowerCase().includes(searchTerm.toLowerCase())
      }
    }), [type, searchTerm])
  )
  
  return (

    <List
      split={false}
      dataSource={uuids}
      renderItem={(uuid) => (
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
