import React, { useCallback } from 'react';

import useStore from '../../stores/Store';


import { Divider, Input, Switch } from 'antd';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';

export const LocationDetail = ({ uuid }) => {

  const { location, setItemProperty } = useStore(useCallback(state => ({
    location: state.data.locations[uuid],
    setItemProperty: state.setItemProperty
  })
    , [uuid]))

  const { TextArea } = Input;

  // const { deleteItem, setItemProperty } = useEvdStore(state=>({
  //     deleteItem:state.deleteItem,
  //     setItemProperty:state.setItemProperty
  // }));

  return (
    <>

      <TextArea
        value={location.description}
        disabled={!location.editable}
      />

      <Divider orientation="left" style={{ color: 'white', borderTopColor: 'rgba(255,255,255,0.12)', lineHeight: '1.5715px', paddingTop: '20px', paddingBottom: '5px' }}>
        Placement:
      </Divider>

      <div style={{ display: 'flex', flexDirection: 'column' }}>
        <PositionInput value={[location.position.x, location.position.y, location.position.z]}
          onChange={e => setItemProperty('location', location.uuid, 'position', { ...location.position, x: e[0], y: e[1], z: e[2] })} />
        <OrientationInput value={[location.orientation.w, location.orientation.x, location.orientation.y, location.orientation.z]}
          onChange={e => setItemProperty('location', location.uuid, 'orientation', { ...location.orientation, w: e[0], x: e[1], y: e[2], z: e[3] })} />
        <br />
        <div style={{ paddingTop: '5px', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <b>Reachable:</b>
          <Switch disabled checked={location.joints.reachable} style={{ left: '-30px' }} />


        </div>
      </div>







    </>

  )
}
