import React, { forwardRef, useCallback } from "react";
import { ParameterZone } from "./ParameterZone";
import useEvdStore from "../../stores/EvdStore";
import useGuiStore from "../../stores/GuiStore";
import blockStyles from "./blockStyles";
import { ItemSortable } from './Wrappers';
import { Card, Button } from 'antd';
import './highlight.css';

export const TrajectoryBlock = forwardRef((props, ref) => {
  const {uuid} = props.data;
  const focused = useGuiStore(useCallback(state=>state.focusItem.uuid === uuid,[uuid]));
  const focusExists = useGuiStore(state=>state.focusItem.type !== null);
  const [frame, clearFocusItem] = useGuiStore(state=>[state.frame,state.clearFocusItem]);
  const unfocused = focusExists && !focused;

  const styles = {
    backgroundColor:
      blockStyles[props.data.type],
    minHeight: 30,
    minWidth: 200,
    borderRadius: 3,
    margin: 4,
    padding: 5,
    position:'relative',
    zIndex:focused?100:1
  };

  const ancestors = [
    {uuid:props.data.uuid},
    ...props.ancestors
];

  return (
    <div {...props} ref={ref} style={{ ...props.style, ...styles }} className={focused?`focus-${frame}`:null} onClick={(e)=>{e.stopPropagation();unfocused&&clearFocusItem()}}>
      <div style={{ fontSize: 16 }}>Trajectory</div>
        <div>
          Start Location:{" "}
          <ParameterZone
            displayText={props.data.start_location_uuid}
            acceptTypes={['uuid-location']}
            itemType="location"
            canRemove={props.data.editable}
            onRemove={() => console.log("delete param")}
            onDrop={(data) => console.log(data)}
          />
        </div>
        <Card 
                title={<span>Waypoints</span>} 
                role="Box" 
                style={{minWidth:250}}
                headStyle={{backgroundColor:blockStyles['node.primitive.hierarchical.skill.']}}
                bodyStyle={{minHeight:30,padding:0,position:'relative'}}
                >
                <div>
                    {props.data.waypoint_uuids.map((id,idx)=>(
                        <ItemSortable key={id} id={id} idx={idx} ancestors={ancestors} itemType='waypoint'/>
                    ))}
                </div>
            </Card>
        <div>
          End Location:{" "}
          <ParameterZone
            displayText={props.data.end_location_uuid}
            acceptTypes={['uuid-location']}
            itemType="location"
            canRemove={props.data.editable}
            onRemove={() => console.log("delete param")}
            onDrop={(data) => console.log(data)}
          />
        </div>
    </div>
  );
});