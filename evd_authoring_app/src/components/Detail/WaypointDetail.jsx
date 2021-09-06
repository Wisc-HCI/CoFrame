import React, {useCallback} from 'react';

import useStore from '../../stores/Store';


import { Space,Divider,Input,Switch } from 'antd';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';

export const WaypointDetail = ({uuid}) => {

    const {waypoint} = useStore(useCallback(state=>({
        waypoint:state.data.waypoints[uuid],

    })
      ,[uuid]))

      const {setItemProperty} = useStore(state=>({
          setItemProperty:state.setItemProperty
      }));

    const { TextArea } = Input;

    // const { deleteItem, setItemProperty } = useStore(state=>({
    //     deleteItem:state.deleteItem,
    //     setItemProperty:state.setItemProperty
    // }));

    console.log(waypoint);

    return (
      <>

      <Space/>

      <TextArea
        defaultValue={waypoint.description}
        disabled = {!waypoint.editable}
      />

      <Divider orientation="left" style={{color:'white',borderTopColor:'rgba(255,255,255,0.12)',lineHeight: '1.5715px',paddingTop:'20px',paddingBottom:'5px'}}>
      Processing:
      </Divider>
      <div style={{display:'flex',flexDirection:'column'}}>
      <PositionInput value={[waypoint.position.x, waypoint.position.y,waypoint.position.z]}
      onChange={e=>setItemProperty('waypoint',waypoint.uuid,'waypoint',{...waypoint.position, x :e[0],y : e[1],z: e[2]})}/>
      <OrientationInput value={[waypoint.orientation.w, waypoint.orientation.x, waypoint.orientation.y,waypoint.orientation.z]}
      onChange={e=>setItemProperty('waypoint',waypoint.uuid,'orientation',{...waypoint.orientation, w:e[0],x :e[1],y : e[2],z: e[3]})}/>
      <br/>
      <div style={{paddingTop: '5px',display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
       <b>Reachable:</b>
       <Switch disabled checked = {waypoint.joints.reachable} style={{left :'-30px' }}/>


      </div>

      </div>



</>

  )
}
