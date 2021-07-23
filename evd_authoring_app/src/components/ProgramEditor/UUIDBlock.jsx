import React, { forwardRef, useCallback } from "react";
import { Tag } from "antd";
import useEvdStore, {typeToKey} from "../../stores/EvdStore";

export const UUIDBlock = forwardRef((props, ref) => {
  
  // props constains data,
  // which contains fields 'itemType' and 'uuid'

  const name = useEvdStore(useCallback(state=>state.data[typeToKey(props.data.itemType)][props.data.uuid].name,[props]));
  
  const itemTypeColorLookup = {
    waypoint: '#AD1FDE',
    thing: '#E08024',
    location: '#8624E0',
    machine: '#B3A533'
}

  return (
    <div {...props} ref={ref} style={props.style}>
      <Tag color={itemTypeColorLookup[props.data.itemType]} closable={false} style={{width:'100%'}}>
          {name}
      </Tag>
    </div>
  );
});
