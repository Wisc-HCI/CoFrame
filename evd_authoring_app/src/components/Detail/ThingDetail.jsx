import React, { useCallback } from 'react';
import useStore from '../../stores/Store';
import { Space, Input } from 'antd';
const { TextArea } = Input;

export const ThingDetail = ({ uuid }) => {

  const thing = useStore(useCallback(state => state.data.thingTypes[uuid], [uuid]));

  return (
    <>
      <p>
        <b>Description:</b>
      </p>
      <Space />

      <TextArea
        value={thing.description}
        disabled={!thing.editable}
      />


    </>

  )
}
