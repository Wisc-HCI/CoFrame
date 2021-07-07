import React, {useCallback} from 'react';

import { Drawer, Input, Empty, Space, Button, Popover } from 'antd';

import useGuiStore from '../../stores/GuiStore';

import { LocationDetail } from './LocationDetail';
import { MachineDetail } from './MachineDetail';
import { ThingDetail } from './ThingDetail';
import { WaypointDetail } from './WaypointDetail';
import { DeleteOutlined } from '@ant-design/icons';
import {MachineRegion} from './MachineRegion'

import useEvdStore from '../../stores/EvdStore';

export const Detail = (_) => {

    const {focusItem, clearFocusItem} = useGuiStore(state=>({
        focusItem:state.focusItem,
        clearFocusItem:state.clearFocusItem
    }));


    const {item} = useEvdStore(useCallback(state=>({
        item:focusItem.type ? state.data[focusItem.type+'s'][focusItem.uuid] : null
    }),[focusItem]))

    const { deleteItem, setItemProperty } = useEvdStore(state=>({
        deleteItem:state.deleteItem,
        setItemProperty:state.setItemProperty
    }));

    const {secondaryFocusItem,clearSecondaryFocusItem} = useGuiStore(
      state => ({
        secondaryFocusItem : state.secondaryFocusItem,
        clearSecondaryFocusItem : state.clearFocusItem,
      })
    )

    const handleOK = () =>{
        clearFocusItem();
        deleteItem(focusItem.type,focusItem.uuid);
    }

    const content =(
      <Button
        danger
        block
        onClick={handleOK}
        icon={<DeleteOutlined/>}
      >
      Delete
      </Button>

    )

    if (item) {
        return (
          <div>
            <Drawer
                title={
                    <Space>
                        <span style={{textTransform:'capitalize'}}>{focusItem.type} </span>
                        <Input
                            defaultValue={item.name}
                            disabled={!item.editable}
                            onChange={e=>setItemProperty(focusItem.type,focusItem.uuid,'name',e.target.value)}/>
                    </Space>}
                visible={focusItem.uuid !== null && focusItem.type !== null}
                onClose={clearFocusItem}
                getContainer={false}
                mask = {false}
                footer={
                  <div>
                  {item.deleteable ? (
                      <Popover title= "Are you sure you want to delete this item?"
                               trigger = "click"
                               placement ="top"
                               content = {content}>
                                  <Button
                                    danger
                                    block
                                    disabled={!item.deleteable}
                                    icon={<DeleteOutlined/>}

                                  />

                        </Popover>
                    ):(
                      <Button
                        danger
                        block
                        disabled={!item.deleteable}
                        icon={<DeleteOutlined/>}
                      />

                    )}


                  </div>
                }
                width='25%'
            >
                {focusItem.type === 'location' && (
                    <LocationDetail uuid={focusItem.uuid}/>
                )}
                {focusItem.type === 'machine' && (
                  <>
                    <div>
                    <MachineDetail uuid={focusItem.uuid} />
                    <Drawer title = "Region"
                            onClose = {clearSecondaryFocusItem}
                            visible = {secondaryFocusItem.uuid !== null && secondaryFocusItem.type !== null}
                            width='20%'
                            placement = 'right'>
                            <MachineRegion/>
                    </Drawer>
                    </div>
                  </>

                )}
                {focusItem.type === 'waypoint' && (
                    <WaypointDetail uuid={focusItem.uuid}/>
                )}
                {focusItem.type === 'thingType' && (
                    <ThingDetail uuid={focusItem.uuid}/>
                )}
                {focusItem.type === null && (
                    <Empty/>
                )}
            </Drawer>
            </div>
        )
    } else {
        return  (
          <Drawer
                  title={<span style={{textTransform:'capitalize'}}>{focusItem.type} </span>}
                  visible={focusItem.uuid !== null && focusItem.type !== null}
                  onClose={clearFocusItem}
                  getContainer={false}
                  width='100%'
              >
              </Drawer>
        )

    }
}
