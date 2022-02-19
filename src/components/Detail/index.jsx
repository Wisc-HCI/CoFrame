import React from 'react';

// import { Drawer, Input, Empty, Space, Button, Popover } from 'antd';
import { Box, Button, Layer, Text, TextInput, DropButton } from 'grommet';
import { FormClose, Trash } from 'grommet-icons';

import { LocationDetail } from './LocationDetail';
import { MachineDetail } from './MachineDetail';
import { ThingDetail } from './ThingDetail';
import { WaypointDetail } from './WaypointDetail';
import { DeleteOutlined } from '@ant-design/icons';
import { MachineInOutRegionDetail } from './MachineInOutRegionDetail'

import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';

const EDITOR_TYPES = ['primitive', 'skill', 'program', 'trajectory', 'scene']
// const IGNORE_ITEMS = ['program','scene','trajectory']



export const Detail = (_) => {

  const {
    focusItem,
    clearFocusItem,
    secondaryFocusItem,
    deleteItem,
    setItemProperty,
    item,
    objectTypeInfo
  } = useStore(state => ({
    focusItem: state.focusItem,
    clearFocusItem: state.clearFocusItem,
    secondaryFocusItem: state.secondaryFocusItem,
    deleteItem: state.deleteItem,
    setItemProperty: state.setItemProperty,
    item: state.focusItem.type ? state.programData[state.focusItem.uuid] : null,
    objectTypeInfo: state.focusItem.type ? state.programSpec.objectTypes[state.programData[state.focusItem.uuid].type] : null
  }),shallow);

  // const handleOK = () => {
  //   clearFocusItem();
  //   deleteItem(focusItem.type, focusItem.uuid);
  // }

  // const content = (
  //   <Button
  //     danger
  //     block
  //     onClick={handleOK}
  //     icon={<DeleteOutlined />}
  //   >
  //     Delete
  //   </Button>

  // )

  console.log({focusItem, item, objectTypeInfo})

  if (focusItem.uuid && !EDITOR_TYPES.includes(focusItem.type)) {
    return (
      <Layer full="vertical" position="right" onEsc={clearFocusItem} modal={false}>
        <Box fill style={{ minWidth: '378px' }} background='#444444'>
          <Box
            direction="row"
            align="center"
            as="header"
            justify="between"
            border={{ side: 'bottom', color: '#333333' }}
          >
            <Text margin={{ left: 'small' }}>
              <span style={{ textTransform: 'capitalize' }}>{objectTypeInfo.name} Information</span>
            </Text>
            <Button icon={<FormClose />} onClick={clearFocusItem} />
          </Box>
          <Box flex overflow="auto" pad="xsmall">
            <TextInput
              placeholder="type here"
              value={item.name}
              disabled={!item.editable}
              onChange={e => setItemProperty(focusItem.type, focusItem.uuid, 'name', e.target.value)}
            />

            {focusItem.type === 'location' && (
              <LocationDetail uuid={focusItem.uuid} />
            )}
            {focusItem.type === 'machine' && (
              <>
                <div>
                  <MachineDetail uuid={focusItem.uuid} />
                  <MachineInOutRegionDetail uuid={secondaryFocusItem.uuid} />
                </div>
              </>

            )}
            {focusItem.type === 'waypoint' && (
              <WaypointDetail uuid={focusItem.uuid} />
            )}
            {focusItem.type === 'thingType' && (
              <ThingDetail uuid={focusItem.uuid} />
            )}

          </Box>
          <Box
            as="footer"
            border={{ side: 'top', color: '#333333' }}
            pad="small"
            justify="end"
            direction="row"

            align="center"
          >
            <div style={{ marginInline: "30%", display: 'flex' }}>
              {item.deleteable ? (
                <div style={{ display: 'flex' }}>
                  <DropButton secondary icon={<Trash />}
                    dropAlign={{ bottom: 'top', right: 'right' }}
                    dropContent={
                      <Box
                        background="grey"
                        pad="small"
                        round="xxsmall"
                        align="center"
                        elevation="none"
                        justify="center">
                        <Text>
                          Are you sure you want to delete this item?
                        </Text>
                        <div style={{ paddingTop: "5%" }}>
                          <Button primary icon={<Trash />} label="Delete" color="#ab4646" />

                        </div>

                      </Box>
                    }

                    disabled={!item.deleteable}

                    label="Delete" color="#ab4646"
                  />
                </div>


              ) : (
                <Button secondary icon={<Trash />} disabled={!item.deleteable} label="Delete" color="#ab4646" />

              )}
              <div style={{ marginInline: "30%", display: 'flex' }}>
                <Button primary label="Save" />
              </div>
            </div>


          </Box>
        </Box>
      </Layer>

    )
  } else {
    return null
  }

  // if (EDITOR_TYPES.indexOf(focusItem.type) < 0 && item) {
  //   return (
  //     <div style={{ color: 'white' }}>
  //       <Drawer
  //         title={
  //           <Space>
  //             <span style={{ textTransform: 'capitalize' }}>{focusItem.type} </span>
  //             <Input
  //               value={item.name}
  //               disabled={!item.editable}
  //               onChange={e => setItemProperty(focusItem.type, focusItem.uuid, 'name', e.target.value)} />
  //           </Space>}
  //         visible={focusItem.uuid !== null && focusItem.type !== null}
  //         onClose={clearFocusItem}
  //         getContainer={false}
  //         mask={false}
  //         footer={
  //           <div>
  //             {item.deleteable ? (
  //               <Popover title="Are you sure you want to delete this item?"
  //                 trigger="click"
  //                 placement="top"
  //                 content={content}>
  //                 <Button
  //                   danger
  //                   block
  //                   disabled={!item.deleteable}
  //                   icon={<DeleteOutlined />}

  //                 />

  //               </Popover>
  //             ) : (
  //               <Button
  //                 danger
  //                 block
  //                 disabled={!item.deleteable}
  //                 icon={<DeleteOutlined />}
  //               />

  //             )}


  //           </div>
  //         }
  //         width='25%'
  //       >
  //         {focusItem.type === 'location' && (
  //           <LocationDetail uuid={focusItem.uuid} />
  //         )}
  //         {focusItem.type === 'machine' && (
  //           <>
  //             <div>
  //               <MachineDetail uuid={focusItem.uuid} />
  //               <MachineInOutRegionDetail uuid={secondaryFocusItem.uuid} />
  //             </div>
  //           </>

  //         )}
  //         {focusItem.type === 'waypoint' && (
  //           <WaypointDetail uuid={focusItem.uuid} />
  //         )}
  //         {focusItem.type === 'thingType' && (
  //           <ThingDetail uuid={focusItem.uuid} />
  //         )}
  //         {focusItem.type === null && (
  //           <Empty />
  //         )}
  //       </Drawer>
  //     </div>
  //   )
  // } else {
  //   return (
  //     <Drawer
  //       title={<span style={{ textTransform: 'capitalize' }}>{focusItem.type} </span>}
  //       visible={focusItem.uuid !== null && focusItem.type !== null && EDITOR_TYPES.indexOf(focusItem.type) < 0}
  //       onClose={clearFocusItem}
  //       getContainer={false}
  //       width='25%'
  //     >
  //     </Drawer>
  //   )

  // }
}
