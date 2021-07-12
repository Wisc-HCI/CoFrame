import React, {useCallback} from 'react';
import useEvdStore from '../../stores/EvdStore';
import { Space, Divider, Input } from 'antd';
const { TextArea } = Input;

export const ThingDetail = ({uuid}) => {
    
    const thing = useEvdStore(useCallback(state=>state.data.thing[uuid],[uuid]));

    return (
      <>
      <p>
      <b>Description:</b>
      </p>
      <Space/>

      <TextArea
        defaultValue={thing.description}
        disabled = {!thing.editable}
      />

      <Divider/>

  <Divider/>


</>

  )
}
