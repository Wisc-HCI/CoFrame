import React, {useCallback} from 'react';

import { Drawer, Input, Empty, Space, Button, Popover } from 'antd';

import useGuiStore from '../../stores/GuiStore';

import { LocationDetail } from './LocationDetail';
import { MachineDetail } from './MachineDetail';
import { ThingDetail } from './ThingDetail';
import { WaypointDetail } from './WaypointDetail';
import { DeleteOutlined } from '@ant-design/icons';
import {MachineInOutRegionDetail} from './MachineInOutRegionDetail'

import useEvdStore from '../../stores/EvdStore';

const EDITOR_TYPES = ['primitive','skill','program','trajectory']

export const Detail = (_) => {

    const {focusItem, clearFocusItem} = useGuiStore(state=>({
        focusItem:state.focusItem,
        clearFocusItem:state.clearFocusItem
    }));
    const {childrenDrawer,clearChildrenDrawer} = useGuiStore(state => ({
      childrenDrawer: state.childrenDrawer,
      clearChildrenDrawer : state.clearChildrenDrawer
    }))

    const {secondaryFocusItem} = useGuiStore(state => ({
      secondaryFocusItem : state.secondaryFocusItem
    }))




    const {item} = useEvdStore(useCallback(state=>({
        item:focusItem.type ? state.data[focusItem.type+'s'][focusItem.uuid] : null
    }),[focusItem]))

    let {childItem} = useEvdStore(useCallback(state=>({
      // Right now, only support secondaryFocusItems that are machines
      childItem:['region'].indexOf(secondaryFocusItem.type)>-1 ? state.data[secondaryFocusItem.type+'s'][secondaryFocusItem.uuid] : null
    }),[secondaryFocusItem]))

    if (!childItem){
      childItem = {
        name : 'nothing',
        editable : false
      }
    }// dummy class

    const { deleteItem, setItemProperty } = useEvdStore(state=>({
        deleteItem:state.deleteItem,
        setItemProperty:state.setItemProperty
    }));


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

    if (EDITOR_TYPES.indexOf(focusItem.type)<0 && item && childItem) {
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
                    <Drawer  title={
                          <Space>
                              <span style={{textTransform:'capitalize'}}>{secondaryFocusItem.type} </span>
                              <Input
                                  defaultValue={childItem.name}
                                  disabled={!childItem.editable}
                                  onChange={e=>setItemProperty(secondaryFocusItem.type,secondaryFocusItem.uuid,'name',e.target.value)}/>
                          </Space>}
                            onClose = {clearChildrenDrawer}
                            visible = {childrenDrawer}
                            width='20%'
                            mask = {false}
                            placement = 'right'>
                            <MachineInOutRegionDetail uuid = {secondaryFocusItem.uuid}/>
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
                  visible={focusItem.uuid !== null && focusItem.type !== null && EDITOR_TYPES.indexOf(focusItem.type)<0}
                  onClose={clearFocusItem}
                  getContainer={false}
                  width='25%'
              >
          </Drawer>
        )

    }
}
