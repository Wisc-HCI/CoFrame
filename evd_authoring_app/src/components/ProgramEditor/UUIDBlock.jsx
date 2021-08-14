import React, { forwardRef, useCallback } from "react";
import { Tag } from "antd";
import useEvdStore, {typeToKey} from "../../stores/EvdStore";
import blockStyles from "./blockStyles";

export const UUIDBlock = forwardRef(({data,preview,style}, ref) => {
  
  // props constains data,
  // which contains fields 'itemType' and 'uuid'
  const {itemType, uuid} = data;

  const name = useEvdStore(useCallback(state=>itemType==='placeholder'?state.data.placeholders[uuid].pending_node.name:state.data[typeToKey(itemType)][uuid].name,[itemType,uuid]));

  return (
    <div ref={preview} style={style}>
      <Tag ref={ref} color={blockStyles[itemType==='placeholder'?'thing':itemType]} closable={false} style={{width:'100%'}}>
          {name}
      </Tag>
    </div>
  );
});
