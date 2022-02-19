import React, { useCallback, useState } from 'react';
import useStore from '../../stores/Store';
//import { Space, Divider, Input, Switch } from 'antd';
import { Toggle } from '../Toggle';
import { TextArea, Text, Box, TextInput } from 'grommet';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';
import debounce from 'lodash.debounce';
//const { TextArea } = Input;

export const WaypointDetail = ({ uuid }) => {

  const waypoint = useStore(useCallback(state => state.data.waypoints[uuid], [uuid]));

  const setItemProperty = useStore(state => state.setItemProperty);
  const requestJointProcessorUpdate = useStore(state => state.requestJointProcessorUpdate);

  const setFocusItem = useStore(state => state.setFocusItem);
  const [activeTransform, setActiveTransform] = useState('inactive');

  const debouncedRequest = debounce(() => {
    requestJointProcessorUpdate('waypoint', uuid)
  }, 1000)

  const positionOnOpen = () => {
    setFocusItem('waypoint', waypoint.uuid, 'translate');
    setActiveTransform('translate');
  }
  const positionOnClose = () => {
    setFocusItem('waypoint', waypoint.uuid, 'inactive');
    setActiveTransform('inactive');
  }
  function orientationOnClose() {
    setFocusItem('waypoint', waypoint.uuid, 'inactive');
    setActiveTransform('inactive');
  }
  function orientationOnOpen() {
    setFocusItem('waypoint', waypoint.uuid, 'rotate');
    setActiveTransform('rotate');
  }
  const onPositionChange = (e) => {
    setItemProperty('waypoint', waypoint.uuid, 'waypoint', { ...waypoint.position, x: e[0], y: e[1], z: e[2] });
    debouncedRequest();
  }
  const onOrientationChange = (e) => {
    setItemProperty('waypoint', waypoint.uuid, 'orientation', { ...waypoint.orientation, w: e[0], x: e[1], y: e[2], z: e[3] })
    debouncedRequest();
  }

  return (
    <>
      <TextArea
        value={waypoint.description}
        disabled={!waypoint.editable}
      />

      <Box orientation="left" style={{ color: 'white', borderTopColor: 'rgba(255,255,255,0.12)', lineHeight: '1.5715px', paddingTop: '20px', paddingBottom: '5px' }}>
        <Text>
          Processing:
        </Text>
      </Box>
      <div style={{ display: 'flex', flexDirection: 'column' }}>
        <PositionInput value={[waypoint.position.x, waypoint.position.y, waypoint.position.z]} onOpen={positionOnOpen} onClose={positionOnClose}
          onChange={onPositionChange} openStatus={activeTransform === 'translate'} />
        <OrientationInput value={[waypoint.orientation.w, waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z]} onOpen={orientationOnOpen} onClose={orientationOnClose}
          onChange={onOrientationChange} openStatus={activeTransform === 'rotate'} />
        <br />
        <div style={{ paddingTop: '5px', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <Text>Reachable:</Text>
          <Toggle checked={waypoint.joints.reachable} style={{ left: '-30px' }} />


        </div>

      </div>



    </>

  )
}
