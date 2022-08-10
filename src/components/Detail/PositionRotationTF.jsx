import React from "react";
// import useStore from '../../stores/Store';
import { Box } from "grommet";
import PositionInput from "./PositionInput";
import RotationInput from "./RotationInput";
import { Card, CardHeader } from "@mui/material";
import useStore from "../../stores/Store";
import shallow from "zustand/shallow";

function PositionRotationTF(props) {
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
        sx={{ padding: '0px 5px 5px 5px' }}
      >
        <CardHeader
          title="Placement"
          titleTypographyProps={{ variant: "subtitle1" }}
        />
        <Box direction="column" gap="small">
          {/* <b style={{ color: 'rgba(255, 255, 255, 0.85)'}} >Placement : </b> */}

          <Box>
            <PositionInput
              itemID={props.itemID}
              position={props.position}
              prevID={props.prevID}
              mode={mode}
              disabled={props.disabled}
            />
          </Box>
          <Box>
            <RotationInput
              itemID={props.itemID}
              rotation={props.rotation}
              prevID={props.prevID}
              mode={mode}
              disabled={props.disabled}
            />
          </Box>
        </Box>
      </Card>
    </>
  );
}

export default PositionRotationTF;
