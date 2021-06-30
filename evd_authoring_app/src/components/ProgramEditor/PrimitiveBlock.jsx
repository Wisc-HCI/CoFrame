import React, { forwardRef } from "react";
import { ParameterZone } from "./ParameterZone";
import { Input } from "antd";

export const PrimitiveBlock = forwardRef((props, ref) => {
  // TODO: Render differently depending on the primitive properties

  const styles = {
    backgroundColor:
      props.data.type === "node.primitive.skill-call." ? "#62869e" : "#629e6c",
    minHeight: 30,
    minWidth: 200,
    borderRadius: 3,
    margin: 4,
    padding: 5,
  };

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
          Duration: <Input size="small" style={{ width: "20%" }} /> seconds
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
            <Input size="small" style={{ width: "20%", margin: "5px" }} /> %
          </div>
          <div>
            Effort:{" "}
            <Input size="small" style={{ width: "20%", margin: "2px" }} /> %
          </div>
          <div>
            Speed:{" "}
            <Input size="small" style={{ width: "20%", margin: "2px" }} /> %
          </div>
        </div>
      )}
    </div>
  );
});
