import React, {useCallback} from 'react';

import useEvdStore from '../../stores/EvdStore';


import { Divider,Input,Switch} from 'antd';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';

export const MachineInOutRegionDetail = ({uuid}) => {
  const {region} = useEvdStore(useCallback(state=>({
      region:state.data.regions[uuid],

  })
    ,[uuid]))

    const { TextArea } = Input;

    const {setItemProperty} = useEvdStore(state=>({
        setItemProperty:state.setItemProperty
    }));
  return (
    <>
    <TextArea
      defaultValue={region.description}
      disabled = {!region.editable}
    />

    <Divider orientation="left" style={{color:'white',borderTopColor:'rgba(255,255,255,0.12)',lineHeight: '1.5715px',paddingTop:'20px',paddingBottom:'5px'}}>
    <span>Placement:</span>
    </Divider>

    <div style={{display:'flex',flexDirection:'column'}}>
    <PositionInput value={[region.center_position.x, region.center_position.y,region.center_position.z]}
    onChange={e=>setItemProperty('region',region.uuid,'center_position',{...region.center_position, x :e[0],y : e[1],z: e[2]})}/>
    <OrientationInput value={[region.center_orientation.w, region.center_orientation.x,region.center_orientation.y,region.center_orientation.z]}
    onChange={e=>setItemProperty('region',region.uuid,'center_orientation',{...region.center_orientation, w:e[0],x :e[1],y : e[2],z: e[3]})}/>
    <br/>
    <div style={{paddingTop: '5px',display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
    <b style ={{color:'rgba(255, 255, 255, 0.85)'}}>Free Orientation:</b>
    <Switch disabled = {!region.editable} checked = {region.free_orientation} style={{left :'-30px' }}/>



    </div>
    <b style ={{color:'rgba(255, 255, 255, 0.85)'}}>Shape:</b>
    <b style ={{color:'rgba(255, 255, 255, 0.85)'}}>Dimension:</b>

    </div>

    </>


  )
}
