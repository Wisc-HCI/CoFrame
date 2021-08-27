import React, { useCallback } from 'react';

import { List, Space, Button, Popover } from 'antd';
import { DeleteOutlined, EllipsisOutlined } from '@ant-design/icons';

import useStore from '../../../stores/Store';


export function Item(props) {

  const { type, uuid, title, description } = props;

  const item = useStore(useCallback(state =>
    state.data[type + 's'][uuid]
    , [uuid, type]))

  const deleteItem = useStore(state => state.deleteItem);

  const { focusItem, setFocusItem, primaryColor } = useStore(state => ({
    focusItem: state.focusItem,
    setFocusItem: state.setFocusItem,
    primaryColor: state.primaryColor
  }));


  const content = (
    <Button
      danger
      block
      onClick={() => deleteItem(type, uuid)}
      icon={<DeleteOutlined />}
    >
      Delete
    </Button>

  )


  return (
    <List.Item
      extra={
        <Space align='center'>
          <Button
            onClick={() => setFocusItem(type, uuid,description)}
            icon={<EllipsisOutlined />}
          />


          <div>
            {item.deleteable ? (
              <Popover title="Are you sure you want to delete this item?"
                trigger="click"
                placement="left"
                content={content}>
                <Button
                  danger
                  disabled={!item.deleteable}
                  icon={<DeleteOutlined />}
                />

              </Popover>
            ) : (
              <Button
                danger
                disabled={!item.deleteable}
                icon={<DeleteOutlined />}
              />

            )}
          </div>

        </Space>

      }

      style={{
        margin: 5, padding: 10,
        borderRadius:4,
        boxShadow: focusItem.type === type && focusItem.uuid === uuid ? `inset 0pt 0pt 1pt 1pt ${primaryColor}` : null,
        backgroundColor: focusItem.type === type && focusItem.uuid === uuid ? `${primaryColor}22` : '#1f1f1f'
      }}
    >
      <List.Item.Meta
        title={title(item)}
        description={description(title)}
      />
    </List.Item>
  );
};
