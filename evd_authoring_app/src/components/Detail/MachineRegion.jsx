import React, {useCallback,useState} from 'react';



import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';


import { List, Space, Button, Popover,InputNumber,Divider,Col,Input,Drawer } from 'antd';
import { DeleteOutlined, EllipsisOutlined,EditOutlined } from '@ant-design/icons';
import {eulerFromQuaternion, quaternionFromEuler} from './Geometry';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';

export const MachineRegion = ({uuid}) => {

  const {region} = useEvdStore(useCallback(state=>({
      region:state.data.regions[uuid],

  })
    ,[uuid]))



return (
  <Button >
  sfesf
  </Button>
)
}
