import React, { forwardRef, useCallback } from "react";
import { Tag } from "antd";
import useEvdStore, {typeToKey} from "../../stores/EvdStore";
import blockStyles from "./blockStyles";
import { ParameterZone } from "./ParameterZone";

export const ExecuteMacrosBlock = forwardRef(({data,style,preview,ancestors,context}, ref) => {
  
  // props constains data,
  // which contains fields 'itemType' and 'uuid'
  const inDrawer = ancestors[0].uuid === 'drawer';
  const editingEnabled = !inDrawer && data.editable;

  // const params = useEvdStore(useCallback(state=>state.data[typeToKey('parameter')][data.uuid],[data.uuid,'parameter']));

  const [setPrimitiveParameter] = useEvdStore(
    (state) => [state.setPrimitiveParameter]
  );

  return (
    <div ref={preview} style={style}>
        <Tag ref={ref} color={blockStyles['node.primitive.skill-call.']} closable={false} style={{width:'100%'}}>
            {data.uuid}
            {Object.entries(data.parameters).map(obj => {
                (obj.type === 'machine' && (<div>
                    <ParameterZone
                        displayText={context[data.parameters.machine_uuid]}
                        acceptTypes={['uuid-machine']}
                        itemType="machine"
                        canRemove={editingEnabled}
                        onRemove={() => setPrimitiveParameter('primitive',data.uuid,'machine_uuid',null)}
                        onDrop={(dropData) => setPrimitiveParameter('primitive',data.uuid,'machine_uuid',dropData.uuid)}
                    /> 
                </div>))
            })}
        </Tag>
    </div>
  );
});
