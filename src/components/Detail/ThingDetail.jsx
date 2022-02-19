import React, { useCallback } from 'react';
import useStore from '../../stores/Store';
import { Space, Input } from 'antd';
import { Toggle } from '../Toggle';
import { TextArea, Text, Box, TextInput } from 'grommet';
//const { TextArea } = Input;

export const ThingDetail = ({ uuid }) => {

  const thing = useStore(useCallback(state => state.data.thingTypes[uuid], [uuid]));

  return (
    <>
      <p>
        <Text>Description:</Text>
      </p>
      

      <TextArea
        value={thing.description}
        disabled={!thing.editable}
      />


    </>

  )
}
