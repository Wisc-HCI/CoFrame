import { React } from "react";
import { MachineProcessList } from "./MachineDetail";
import { ProcessIOList } from "./ProcessDetail";
import PositionRotationTF from "./PositionRotationTF";
import {
  TextArea,
  Text,
  Box,
  TextInput,
  Button,
  Stack,
  DropButton,
  Spinner,
} from "grommet";
import { FixtureItem } from "./FixtureDetail";
import { GizmoDetail } from "./GizmoDetail";

import useStore from "../../stores/Store";
import shallow from "zustand/shallow";
import { FiTrash, FiX, FiSquare } from "react-icons/fi";
import { NumberInput } from "../Elements/NumberInput";
import { DETAIL_TYPES, STATUS } from "../../stores/Constants";
import JointGripperInput from "./JointGripperInput";
import LocationWaypointDetail from "./LocationWaypointDetail";
import { ScrollRegion } from "../Elements/ScrollRegion";
import {
  Drawer,
  Card,
  IconButton,
  TextField,
  LinearProgress,
} from "@mui/material";

export const Detail = (_) => {
  const { item, objectTypeInfo } = useStore((state) => {
    let focusData = state.focus
      .map((f) => {
        if (state.programData[f]) {
          return state.programData[f];
        } else if (state.issues[f]) {
          return state.issues[f];
        } else {
          return null;
        }
      })
      .filter((d) => d !== null);

    let item = null;

    state.focus
      .slice()
      .reverse()
      .some((v) => {
        if (
          state.programData[v] &&
          state.activeFocus === v &&
          DETAIL_TYPES.includes(state.programData[v].type)
        ) {
          item = state.programData[v];

          return true;
        } else {
          return false;
        }
      });

    if (item === null && focusData.length > 0) {
      if (
        state.programData[focusData[0].id] &&
        DETAIL_TYPES.includes(state.programData[focusData[0].id].type)
      ) {
        item = state.programData[focusData[0].id];
      }
    }

    return {
      item,
      objectTypeInfo: item?.type
        ? state.programSpec.objectTypes[state.programData[item?.id].type]
        : null,
    };
  }, shallow);

  //const addFocusItem = useStore(state => state.addFocusItem);
  const clearFocus = useStore((state) => state.clearFocus);
  const updateItemName = useStore((state) => state.updateItemName);
  const updateItemSimpleProperty = useStore(
    (state) => state.updateItemSimpleProperty
  );
  const updateItemDescription = useStore(
    (state) => state.updateItemDescription
  );

  const Icon = objectTypeInfo?.instanceBlock?.icon
    ? objectTypeInfo.instanceBlock.icon
    : objectTypeInfo?.referenceBlock?.icon
    ? objectTypeInfo?.referenceBlock.icon
    : FiSquare;

  const objectColor = objectTypeInfo?.instanceBlock?.color
    ? objectTypeInfo.instanceBlock.color
    : objectTypeInfo?.referenceBlock?.color
    ? objectTypeInfo?.referenceBlock.color
    : "#333333";

  const deleteBlock = useStore((state) => state.deleteBlock);
  //console.log("item:", item);
  // console.log("objectTypeInfo", objectTypeInfo);
  // console.log("focusData", focusData);
  if (!item) {
    return null;
  } else {
    return (
      // <Layer full="vertical" onEsc={clearFocus} position="right" modal={false}>
      <Drawer
        hideBackdrop
        elevation={16}
        open={item !== null && item !== undefined}
        anchor="right"
        variant="persistent"
      >
        <Box
          direction="row"
          align="center"
          as="header"
          justify="between"
          background={objectColor}
          pad="xsmall"
          gap="xsmall"
          border={{ side: "bottom", color: "#333333" }}
        >
          <Box
            align="center"
            justify="center"
            style={{
              backgroundColor: "#22222299",
              color: "white",
              padding: 9,
              borderRadius: 5,
              boxShadow: `0 0 0 1px #dddddd55`,
              height: "39px",
              width: "39px",
            }}
          >
            <Icon />
          </Box>
          
          <TextField
            // label='Name'
            size="small"
            margin="none"
            variant="outlined"
            color="quiet"
            disabled={!item.canEdit}
            value={item.name ? item.name : ""}
            onChange={(e) => {
              updateItemName(item.id, e.target.value);
            }}
            InputProps={{
              style: {
                borderRadius: 5,
                backgroundColor: "#22222299",
              },
            }}
          />
          <Box direction="row" flex />
          <IconButton onClick={clearFocus}><FiX/></IconButton>
        </Box>
        <Box height='5px'>
          {item.properties.status === STATUS.PENDING && <LinearProgress variant="indeterminate" color="primaryColor" />}
        </Box>
        
        <Box
          flex
          overflow="auto"
          pad="xsmall"
          gap="small"
          border={{ color: "black", size: "xxsmall" }}
        >

          <ScrollRegion vertical>
            <Box width="355px" gap="xsmall" pad={{top:'small'}}>

              {/* Description */}
              <TextField 
                multiline
                color='primaryColor'
                label='Description'
                value={item.properties.description}
                onChange={(e) =>
                  updateItemDescription(item.id, e.target.value)
                }
              />

              {item.properties.processTime !== undefined && (
                <Box
                  direction="row"
                  background="#303030"
                  round="xsmall"
                  pad="small"
                  style={{ marginBottom: 5 }}
                  justify="between"
                  wrap={true}
                >
                  <b style={{ color: "rgba(255, 255, 255, 0.85)" }}>Time : </b>
                  <Box direction="row">
                    <NumberInput
                      value={item.properties.processTime}
                      min={0}
                      max={Infinity}
                      onChange={(value) =>
                        updateItemSimpleProperty(item.id, "processTime", value)
                      }
                      disabled={!item.canDelete}
                      visualScaling={1 / 1000}
                    />
                    <b
                      style={{
                        color: "rgba(255, 255, 255, 0.85)",
                        paddingLeft: "4%",
                      }}
                    >
                      sec
                    </b>
                  </Box>
                </Box>
              )}

              {item.properties.position !== undefined &&
                item.properties.rotation !== undefined && (
                  <PositionRotationTF
                        itemID={item.id}
                        disabled={!item.canEdit}
                        position={item.properties.position}
                        rotation={item.properties.rotation}
                      />
                )}

              {item.type === "machineType" && (
                <>
                  <MachineProcessList machineId={item.id} />
                </>
              )}

              {item.type === "processType" && (
                <>
                  <ProcessIOList processId={item.id} isInput />
                  <div style={{ marginBottom: 10 }}></div>
                  <ProcessIOList processId={item.id} />
                </>
              )}
              {item.properties.relativeTo !== undefined &&
                item.properties.relativeTo !== "world" &&
                item.properties.relativeTo !== null && (
                  <>
                    <Box>
                      <FixtureItem fixtureID={item.properties.relativeTo} />
                    </Box>
                  </>
                )}

              {item.type === "gripperType" && (
                <>
                  <JointGripperInput robotID={item.id} isGripper={true} />
                </>
              )}
              {item.type === "robotAgentType" && (
                <>
                  <JointGripperInput robotID={item.id} isGripper={false} />
                </>
              )}
              {(item.type === "locationType" ||
                item.type === "waypointType") && (
                <>
                  <LocationWaypointDetail itemID={item.id} />
                </>
              )}
              {item.properties.gizmo && (
                <>
                  <GizmoDetail gizmoId={item.properties.gizmo} />
                </>
              )}
            </Box>
          </ScrollRegion>
        </Box>
        <Box
          as="footer"
          border={{ side: "top", color: "#333333" }}
          pad="small"
          justify="end"
          direction="row"
          align="center"
        >
          <div style={{ marginInline: "30%", display: "flex" }}>
            {item.canDelete ? (
              <div style={{ display: "flex" }}>
                <DropButton
                  secondary
                  icon={<FiTrash />}
                  dropAlign={{ bottom: "top", right: "right" }}
                  dropProps={{ elevation: "none" }}
                  dropContent={
                    <Box
                      background="grey"
                      pad="small"
                      round="xxsmall"
                      border={{ color: "white", size: "xsmall" }}
                      align="center"
                      elevation="none"
                      justify="center"
                    >
                      <Text>Are you sure you want to delete this item?</Text>
                      <div style={{ paddingTop: "5%" }}>
                        <Button
                          primary
                          icon={<FiTrash />}
                          label="Delete"
                          color="#ab4646"
                          onClick={() =>
                            deleteBlock(item, "spawner", objectTypeInfo)
                          }
                        />
                      </div>
                    </Box>
                  }
                  disabled={!item.canDelete}
                  label="Delete"
                  color="#ab4646"
                />
              </div>
            ) : (
              <Button
                secondary
                icon={<FiTrash />}
                disabled={!item.canDelete}
                label="Delete"
                color="#ab4646"
              />
            )}
          </div>
        </Box>
        {/* </Layer> */}
      </Drawer>
    );
  }
};
