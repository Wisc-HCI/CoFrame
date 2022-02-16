import React from 'react';

// import { Drawer, Input, Empty, Space, Button, Popover } from 'antd';
import { Box, Button, Layer, Text } from 'grommet';
import { FormClose } from 'grommet-icons';

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
    secondaryFocusItem,
    deleteItem,
    setItemProperty,
    item
  } = useStore(state => ({
    focusItem: state.focusItem,
    clearFocusItem: state.clearFocusItem,
    secondaryFocusItem: state.secondaryFocusItem,
    deleteItem: state.deleteItem,
    setItemProperty: state.setItemProperty,
    item: state.focusItem.type && EDITOR_TYPES.indexOf(state.focusItem.type) < 0 ? state.data[state.focusItem.type + 's'][state.focusItem.uuid] : null
  }));

  const handleOK = () => {
    clearFocusItem();
    deleteItem(focusItem.type, focusItem.uuid);
  }

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

  console.log(focusItem)
  
  if (!EDITOR_TYPES.includes(focusItem.type)) {
    return (
      <Layer full="vertical" position="right" onClickOutside={clearFocusItem} onEsc={clearFocusItem}>
        <Box fill style={{ minWidth: '378px' }} background='#444444'>
          <Box
            direction="row"
            align="center"
            as="header"
            justify="between"
            border={{ side: 'bottom', color: '#333333' }}
          >
            <Text margin={{ left: 'small' }}>Header</Text>
            <Button icon={<FormClose />} onClick={clearFocusItem}/>
          </Box>
          <Box flex overflow="auto" pad="xsmall">
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
            <span>body</span>
          </Box>
          <Box
            as="footer"
            border={{ side: 'top', color: '#333333' }}
            pad="small"
            justify="end"
            direction="row"
            align="center"
          >
            <Button primary label="Save" />
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
