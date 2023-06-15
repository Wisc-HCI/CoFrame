import React from "react";
import PositionInput from "./PositionInput";
import RotationInput from "./RotationInput";
import { Card, CardHeader, Stack } from "@mui/material";
import useStore from "../../stores/Store";
import { shallow } from 'zustand/shallow';
import VectorInput from "../Elements/VectorInput";

function PositionRotationTF(props) {

  const updateItemSimpleProperty = useStore(
    (state) => state.updateItemSimpleProperty
  );
  const addFocusItem = useStore((state) => state.addFocusItem);

  function buttonClick(field) {
    addFocusItem(field, true);
  }

  function handleClose() {
    addFocusItem(props.itemID, true);
  }

  const mode = useStore((state) => {
    if (state.focus[state.focus.length - 1] === "translate") {
      return "translate";
    } else if (state.focus[state.focus.length - 1] === "rotate") {
      return "rotate";
    } else {
      return null;
    }
  }, shallow);



  return (
    <>
      <Card
        raised
        // variant='outlined'
        background="#303030"
        sx={{ padding: "0px 5px 5px 5px" }}
      >
        <CardHeader
          title="Placement"
          titleTypographyProps={{ variant: "subtitle1" }}
        />
        <Stack direction="column" spacing={1}>
          {/* <b style={{ color: 'rgba(255, 255, 255, 0.85)'}} >Placement : </b> */}
          <VectorInput
            label="Position"
            value={[
              props.position.x, 
              props.position.y, 
              props.position.z
            ]}
            onChange={(v) => {
              // console.log("onChangeHandle",v)
              updateItemSimpleProperty(props.itemID,'position',{x:v[0], y:v[1], z:v[2]});
            }}
            active={mode === "translate"}
            onToggleActivity={(a) => {
              console.log("onToggleActivity",a)
              if (a) {
                buttonClick('translate')
              } else {
                handleClose()
              }
            }}
          />

          <VectorInput
            label="Rotation"
            value={[
              props.rotation.x, 
              props.rotation.y, 
              props.rotation.z
            ]}
            onChange={(v) => {
              console.log("onChangeHandle",v)
              updateItemSimpleProperty(props.itemID,'rotation',{x:v[0], y:v[1], z:v[2]});
            }}
            active={mode === "rotate"}
            onToggleActivity={(a) => {
              console.log("onToggleActivity",a)
              if (a) {
                buttonClick('rotate')
              } else {
                handleClose()
              }
            }}
          />

          {/* <PositionInput
            itemID={props.itemID}
            position={props.position}
            prevID={props.prevID}
            mode={mode}
            disabled={props.disabled}
          />
          <RotationInput
            itemID={props.itemID}
            rotation={props.rotation}
            prevID={props.prevID}
            mode={mode}
            disabled={props.disabled}
          /> */}
        </Stack>
      </Card>
    </>
  );
}

export default PositionRotationTF;
