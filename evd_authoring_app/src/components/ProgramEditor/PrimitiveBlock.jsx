import React, { forwardRef } from "react";
import { ParameterZone } from "./ParameterZone";
import { InputNumber } from "antd";
import useEvdStore from "../../stores/EvdStore";

export const PrimitiveBlock = forwardRef((props, ref) => {
  const styles = {
    backgroundColor:
      props.data.type === "node.primitive.skill-call." ? "#62869e" : "#629e6c",
    minHeight: 30,
    minWidth: 200,
    borderRadius: 3,
    margin: 4,
    padding: 5,
  };

  const setPrimitiveParameter = useEvdStore(
    (state) => state.setPrimitiveParameter
  );

  if (props.data.parameters.machine_uuid) {
    console.log("found one");
    console.log(props.data.parameters.machine_uuid);
  }

  return (
    <div {...props} ref={ref} style={{ ...props.style, ...styles }}>
      <div style={{ fontSize: 16 }}>{props.data.name}</div>
      {props.data.type.includes("node.primitive.machine-primitive") && (
        <div>
          Machine:{" "}
          <ParameterZone
            displayText={props.data.parameters.machine_uuid}
            acceptTypes={["node.machine."]}
            itemType="machine"
            canRemove={props.data.editable}
            onRemove={() => console.log("delete param")}
            onDrop={(data) => console.log(data)}
          />
        </div>
      )}
      {props.data.type.includes("node.primitive.move-trajectory") && (
        <div>
          Trajectory:{" "}
          <ParameterZone
            displayText={props.data.parameters.trajectory_uuid}
            acceptTypes={["node.trajectory."]}
            itemType="trajectory"
            canRemove={props.data.editable}
            onRemove={() => console.log("delete param")}
            onDrop={(data) => console.log(data)}
          />
        </div>
      )}
      {props.data.type.includes("node.primitive.move-unplanned") && (
        <div>
          To Location:{" "}
          <ParameterZone
            displayText={props.data.parameters.location_uuid}
            acceptTypes={["node.pose.waypoint.location."]}
            itemType="location"
            canRemove={props.data.editable}
            onRemove={() => console.log("delete param")}
            onDrop={(data) => console.log(data)}
          />
        </div>
      )}
      {props.data.type.includes("node.primitive.delay") && (
        <div>
          Duration:{" "}
          <InputNumber
            size="small"
            defaultValue={props.data.parameters.duration}
            style={{ width: "25%", margin: "5px" }}
            onChange={(value) => {
              setPrimitiveParameter(
                "primitive",
                props.data.uuid,
                "duration",
                value
              );
              //console.log(props.data.parameters.duration);
            }}
          />{" "}
          seconds
        </div>
      )}
      {props.data.type.includes("node.primitive.gripper") && (
        <div>
          Thing:{" "}
          <ParameterZone
            displayText={props.data.parameters.thing_uuid}
            acceptTypes={["node.thing-type."]}
            itemType="thing"
            canRemove={props.data.editable}
            onRemove={() => console.log("delete param")}
            onDrop={(data) => console.log(data)}
          />
          <div>
            Position:{" "}
            <InputNumber
              size="small"
              min={0}
              max={100}
              defaultValue={props.data.parameters.position}
              style={{ width: "25%", margin: "5px" }}
              onChange={(value) => {
                setPrimitiveParameter(
                  "primitive",
                  props.data.uuid,
                  "position",
                  value
                );
                // console.log(props.data.parameters.position);
              }}
            />{" "}
            %
          </div>
          <div>
            Effort:{" "}
            <InputNumber
              size="small"
              min={0}
              max={100}
              defaultValue={props.data.parameters.effort}
              style={{ width: "25%", margin: "2px" }}
              onChange={(value) => {
                setPrimitiveParameter(
                  "primitive",
                  props.data.uuid,
                  "effort",
                  value
                );
                // console.log(props.data.parameters.effort);
              }}
            />{" "}
            %
          </div>
          <div>
            Speed:{" "}
            <InputNumber
              size="small"
              min={0}
              max={100}
              defaultValue={props.data.parameters.speed}
              style={{ width: "25%", margin: "2px" }}
              onChange={(value) => {
                setPrimitiveParameter(
                  "primitive",
                  props.data.uuid,
                  "speed",
                  value
                );
                // console.log(props.data.parameters.speed);
              }}
            />{" "}
            %
          </div>
        </div>
      )}
    </div>
  );
});
