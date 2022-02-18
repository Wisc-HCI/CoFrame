import React, { useCallback,useState } from 'react';

import useStore from '../../stores/Store';

import debounce from 'lodash.debounce';
//import { Divider, Input, Switch } from 'antd';
import {Toggle} from '../Toggle';
import { TextArea,Text } from 'grommet';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';

export const LocationDetail = ({ uuid }) => {

  const { location, setItemProperty } = useStore(useCallback(state => ({
    location: state.data.locations[uuid],
    setItemProperty: state.setItemProperty
  })
    , [uuid]))

  //const { TextArea } = Input;
  
  // const { deleteItem, setItemProperty } = useEvdStore(state=>({
  //     deleteItem:state.deleteItem,
  //     setItemProperty:state.setItemProperty
  // }));
  const setFocusItem = useStore(state=>state.setFocusItem);
  const [activeTransform,setActiveTransform] = useState('inactive');
  const requestJointProcessorUpdate = useStore(state => state.requestJointProcessorUpdate);

  const debouncedRequest = debounce(()=>{
    requestJointProcessorUpdate('waypoint',uuid)
  },1000)

  const positionOnOpen = () => {
    setFocusItem('location',location.uuid,'translate');
    setActiveTransform('translate');  
  }
  const positionOnClose = () => {
    setFocusItem('location',location.uuid,'inactive');
    setActiveTransform('inactive');
  }
  function orientationOnClose(){
    setFocusItem('location',location.uuid,'inactive');
    setActiveTransform('inactive');
  }
  function orientationOnOpen(){
    setFocusItem('location',location.uuid,'rotate');
    setActiveTransform('rotate');
  }
  const onPositionChange = (e) => {
    setItemProperty('location',location.uuid,'location',{...location.position, x :e[0],y : e[1],z: e[2]});
    debouncedRequest();
  }
  const onOrientationChange = (e) => {
    setItemProperty('location',location.uuid,'orientation',{...location.orientation, w:e[0],x :e[1],y : e[2],z: e[3]})
    debouncedRequest();
  }

  return (
    <>
      <TextArea
        value={location.description}
        disabled={!location.editable}
      />

      {/* <Divider 
      orientation="left" style={{ color: 'white', borderTopColor: 'rgba(255,255,255,0.12)', lineHeight: '1.5715px', paddingTop: '20px', paddingBottom: '5px' }}>
        Placement:
      </Divider> */}
      <Text>
        Placement:
      </Text>

      <div style={{ display: 'flex', flexDirection: 'column' }}>
        <PositionInput value={[location.position.x, location.position.y, location.position.z]} onOpen = {positionOnOpen} onClose = {positionOnClose}
          onChange={onPositionChange} openStatus = {activeTransform === 'translate'}/>
        <OrientationInput value={[location.orientation.w, location.orientation.x, location.orientation.y, location.orientation.z]} onOpen = {orientationOnOpen} onClose = {orientationOnClose}
          onChange={onOrientationChange} openStatus = {activeTransform === 'rotate'}/>
        <br />
        <div style={{ paddingTop: '5px', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <b>Reachable:</b>
          <Toggle selected={location.joints.reachable} style={{ left: '-30px' }} />
        </div>
      </div>







    </>

  )
}
