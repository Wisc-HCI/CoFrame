import React from "react";
import useStore from "../../stores/Store";
import { Collapse } from "../Elements/Collapse";
// import { Toggle } from "../Elements/Toggle";
import { Switch, IconButton, Stack, Typography } from "@mui/material";
import { FiRefreshCw } from "react-icons/fi";

function LocationWaypointDetail(props) {
  const item = useStore((state) => state.programData[props.itemID]);
  const robotAgents = useStore((state) =>
    Object.values(state.programData).filter(
      (item) => item.type === "robotAgentType"
    )
  );
  const grippers = useStore((state) =>
    Object.values(state.programData).filter(
      (item) => item.type === "gripperType"
    )
  );
  const primaryColor = useStore((state) => state.primaryColor);
  const forceRefreshBlock = useStore((state) => state.forceRefreshBlock);

  return (
    <Collapse
      defaultOpen={true}
      header="Reachability"
      extra={
        <IconButton
          sx={{ marginRight: 1 }}
          size="small"
          color="primaryColor"
          onClick={() => forceRefreshBlock(item.id)}
        >
          <FiRefreshCw color={primaryColor} />
        </IconButton>
      }
    >
      {Object.keys(item.properties.reachability).length === 0 && (
        <Typography>Reachability Loading...</Typography>
      )}
      {robotAgents
        .filter((robotAgent) => item.properties.reachability[robotAgent.id])
        .map((robotAgent) => (
          <Stack
            key={robotAgent.id}
            style={{
              backgroundColor: "black",
              borderRadius: 2,
              padding: 1,
            }}
            spacing={0.5}
          >
            <Typography>{robotAgent.name}</Typography>
            {grippers
              .filter(
                (gripper) =>
                  item.properties.reachability[robotAgent.id][gripper.id] !==
                  undefined
              )
              .map((gripper) => (
                <Stack
                  direction="row"
                  style={{
                    justify: "between",
                    align: "center",
                    borderRadius: 2,
                    backgroundColor: "#333333",
                    padding: 1,
                  }}
                  key={gripper.id}
                  spacing={0.5}
                >
                  <Typography>{gripper.name}</Typography>
                  <Switch
                    onClick={() => {}}
                    checked={
                      item.properties.reachability[robotAgent.id][gripper.id]
                    }
                    size="small"
                    color="primaryColor"
                  />
                </Stack>
              ))}
          </Stack>
        ))}
    </Collapse>
  );
}

export default LocationWaypointDetail;
