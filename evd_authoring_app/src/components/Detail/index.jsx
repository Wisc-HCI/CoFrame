import React from 'react';

import { Drawer, Input, Empty, Space, Button, Popover } from 'antd';

import { LocationDetail } from './LocationDetail';
import { MachineDetail } from './MachineDetail';
import { ThingDetail } from './ThingDetail';
import { WaypointDetail } from './WaypointDetail';
import { DeleteOutlined } from '@ant-design/icons';
import { MachineInOutRegionDetail } from './MachineInOutRegionDetail'

import useStore from '../../stores/Store';

const EDITOR_TYPES = ['primitive', 'skill', 'program', 'trajectory', 'scene']
// const IGNORE_ITEMS = ['program','scene','trajectory']

export const Detail = (_) => {

  const {
    focusItem,
    clearFocusItem,
    childrenDrawer,
    clearChildrenDrawer,
    secondaryFocusItem,
    deleteItem,
    setItemProperty,
    item,
    childItem
  } = useStore(state => ({
    focusItem: state.focusItem,
    clearFocusItem: state.clearFocusItem,
    childrenDrawer: state.childrenDrawer,
    clearChildrenDrawer: state.clearChildrenDrawer,
    secondaryFocusItem: state.secondaryFocusItem,
    deleteItem: state.deleteItem,
    setItemProperty: state.setItemProperty,
    item: state.focusItem.type && EDITOR_TYPES.indexOf(state.focusItem.type) < 0 ? state.data[state.focusItem.type + 's'][state.focusItem.uuid] : null,
    childItem: ['region'].indexOf(state.secondaryFocusItem.type) > -1 ? state.data[state.secondaryFocusItem.type + 's'][state.secondaryFocusItem.uuid] : { name: 'nothing', editable: false }
  }));

  const handleOK = () => {
    clearFocusItem();
    deleteItem(focusItem.type, focusItem.uuid);
  }

  const content = (
    <Button
      danger
      block
      onClick={handleOK}
      icon={<DeleteOutlined />}
    >
      Delete
    </Button>

  )

  if (EDITOR_TYPES.indexOf(focusItem.type) < 0 && item && childItem) {
    return (
      <div style={{ color: 'white' }}>
        <Drawer
          title={
            <Space>
              <span style={{ textTransform: 'capitalize' }}>{focusItem.type} </span>
              <Input
                defaultValue={item.name}
                disabled={!item.editable}
                onChange={e => setItemProperty(focusItem.type, focusItem.uuid, 'name', e.target.value)} />
            </Space>}
          visible={focusItem.uuid !== null && focusItem.type !== null}
          onClose={clearFocusItem}
          getContainer={false}
          mask={false}
          footer={
            <div>
              {item.deleteable ? (
                <Popover title="Are you sure you want to delete this item?"
                  trigger="click"
                  placement="top"
                  content={content}>
                  <Button
                    danger
                    block
                    disabled={!item.deleteable}
                    icon={<DeleteOutlined />}

                  />

                </Popover>
              ) : (
                <Button
                  danger
                  block
                  disabled={!item.deleteable}
                  icon={<DeleteOutlined />}
                />

              )}


            </div>
          }
          width='25%'
        >
          {focusItem.type === 'location' && (
            <LocationDetail uuid={focusItem.uuid} />
          )}
          {focusItem.type === 'machine' && (
            <>
              <div>
                <MachineDetail uuid={focusItem.uuid} />
                <Drawer title={
                  <Space>
                    <span style={{ textTransform: 'capitalize' }}>{secondaryFocusItem.type} </span>
                    <Input
                      defaultValue={childItem.name}
                      disabled={!childItem.editable}
                      onChange={e => setItemProperty(secondaryFocusItem.type, secondaryFocusItem.uuid, 'name', e.target.value)} />
                  </Space>}
                  onClose={clearChildrenDrawer}
                  visible={childrenDrawer}
                  width='20%'
                  mask={false}
                  placement='right'>
                  <MachineInOutRegionDetail uuid={secondaryFocusItem.uuid} />
                </Drawer>
              </div>
            </>

          )}
          {focusItem.type === 'waypoint' && (
            <WaypointDetail uuid={focusItem.uuid} />
          )}
          {focusItem.type === 'thingType' && (
            <ThingDetail uuid={focusItem.uuid} />
          )}
          {focusItem.type === null && (
            <Empty />
          )}
        </Drawer>
      </div>
    )
  } else {
    return (
      <Drawer
        title={<span style={{ textTransform: 'capitalize' }}>{focusItem.type} </span>}
        visible={focusItem.uuid !== null && focusItem.type !== null && EDITOR_TYPES.indexOf(focusItem.type) < 0}
        onClose={clearFocusItem}
        getContainer={false}
        width='25%'
      >
      </Drawer>
    )

  }
}
