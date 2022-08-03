import React from "react";
import useStore from "../../stores/Store";
import { Box, Button } from "grommet";
import Collapse from "../Elements/Collapse";
// import { Toggle } from "../Elements/Toggle";
import { Switch, IconButton } from "@mui/material";
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
      openable={true}
      borderWidth={10}
      defaultOpen={true}
      style={{ backgroundColor: "#303030", marginBottom: 5 }}
      backgroundColor="#202020"
      header={
        <Box direction="row">
          <b style={{ color: "rgba(255, 255, 255, 0.85)" }}>Reachability : </b>
        </Box>
      }
      extra={
        <IconButton
          sx={{marginRight:1}}
          size='small'
          color='primaryColor'
          onClick={() => forceRefreshBlock(item.id)}
        >
          <FiRefreshCw color={primaryColor} />
        </IconButton>
      }
    >
      {Object.keys(item.properties.reachability).length === 0 && (
        <i>Reachability Loading...</i>
      )}
      {robotAgents
        .filter((robotAgent) => item.properties.reachability[robotAgent.id])
        .map((robotAgent) => (
          <Box
            key={robotAgent.id}
            background="black"
            round="xsmall"
            gap="xsmall"
            pad="xsmall"
          >
            {robotAgent.name}
            {grippers
              .filter(
                (gripper) =>
                  item.properties.reachability[robotAgent.id][gripper.id] !==
                  undefined
              )
              .map((gripper) => (
                <Box
                  direction="row"
                  justify="between"
                  align="center"
                  key={gripper.id}
                  background="#333333"
                  round="xsmall"
                  gap="xsmall"
                  pad="xsmall"
                >
                  {gripper.name}
                  <Switch onClick={()=>{}} checked={item.properties.reachability[robotAgent.id][gripper.id]} size="small" color='primaryColor'/>
                  {/* <Toggle
                    selected={
                      item.properties.reachability[robotAgent.id][gripper.id]
                    }
                    disabled={true}
                    backgroundColor={primaryColor}
                    size="small"
                  /> */}
                </Box>
              ))}
          </Box>
        ))}
    </Collapse>
  );
}

export default LocationWaypointDetail;
