import React, { useState } from "react";
import PositionRotationTF from "./PositionRotationTF";
import { Box } from "grommet";
import { GizmoDetail } from "./GizmoDetail";

import useStore from "../../stores/Store";
import shallow from "zustand/shallow";
import { FiX, FiSquare, FiTrash2 } from "react-icons/fi";
import { NumberInput } from "../Elements/NumberInput";
import { DETAIL_TYPES, STATUS } from "../../stores/Constants";
import JointGripperInput from "./JointGripperInput";
import LocationWaypointDetail from "./LocationWaypointDetail";
import { ScrollRegion } from "../Elements/ScrollRegion";
import {
  Drawer,
  IconButton,
  TextField,
  LinearProgress,
  Dialog,
  DialogTitle,
  Button,
  Typography,
  DialogContent,
  DialogActions,
} from "@mui/material";
import { BackRefSection } from "./BackRefSection";
import { ForwardRefSection } from "./ForwardRefSection";

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
  const clearFocus = useStore((state) => state.clearFocus, shallow);
  const updateItemName = useStore((state) => state.updateItemName, shallow);
  const updateItemSimpleProperty = useStore(
    (state) => state.updateItemSimpleProperty,
    shallow
  );
  const updateItemDescription = useStore(
    (state) => state.updateItemDescription,
    shallow
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

  const deleteBlock = useStore((state) => state.deleteBlock, shallow);
  //console.log("item:", item);
  // console.log("objectTypeInfo", objectTypeInfo);
  // console.log("focusData", focusData);

  const [deleteConfirmOpen, setDeleteConfirmOpen] = useState(false);

  return (
    // <Layer full="vertical" onEsc={clearFocus} position="right" modal={false}>
    <Drawer
      anchor="right"
      sx={{
        width: 375,
        height: "100vh",
        flexShrink: 0,
        "& .MuiDrawer-paper": {
          width: 375,
          boxSizing: "border-box",
        },
      }}
      variant="persistent"
      open={item !== null && item !== undefined}
      hideBackdrop
    // elevation={16}
    >

      {item && (
        <>
          <Box
            direction="row"
            align="center"
            as="header"
            justify="between"
            background={objectColor}
            pad="xsmall"
            gap="xsmall"
          // border={{ side: "bottom", color: "#333333" }}
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
            <IconButton onClick={clearFocus}>
              <FiX />
            </IconButton>
          </Box>
          <Box height="5px">
            {item.properties.status === STATUS.PENDING && (
              <LinearProgress variant="indeterminate" color="primaryColor" />
            )}
          </Box>

          <Box
            flex
            overflow="auto"
            pad="xsmall"
            gap="small"
          // border={{ color: "black", size: "xxsmall" }}
          >
            <ScrollRegion vertical>
              <Box width="355px" gap="xsmall" pad={{ top: "small" }}>
                {/* Description */}
                <TextField
                  multiline
                  color="primaryColor"
                  label="Description"
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
                    <b style={{ color: "rgba(255, 255, 255, 0.85)" }}>
                      Time :{" "}
                    </b>
                    <Box direction="row">
                      <NumberInput
                        value={item.properties.processTime}
                        min={0}
                        max={Infinity}
                        onChange={(value) =>
                          updateItemSimpleProperty(
                            item.id,
                            "processTime",
                            value
                          )
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

                {(item.type === "machineType" || item.type === "toolType") && (
                  <BackRefSection
                    title="Processes"
                    reference={item.id}
                    targetType="processType"
                    targetField="gizmo"
                  />
                )}

                {/* {item.type === "machineType" && (
              <>
                <MachineProcessList machineId={item.id} />
              </>
            )} */}

                {item?.properties?.relativeTo && (
                  <ForwardRefSection
                    references={
                      item.properties.relativeTo === "world"
                        ? []
                        : [item.properties.relativeTo]
                    }
                    title="Relative To"
                  />
                )}

                {item?.properties?.graspPoints && (
                  <ForwardRefSection
                    references={item.properties.graspPoints}
                    title="Grasp Points"
                  />
                )}
                {item?.properties?.inputs && (
                  <ForwardRefSection
                    references={item.properties.inputs}
                    title="Inputs"
                  />
                )}
                {item?.properties?.outputs && (
                  <ForwardRefSection
                    references={item.properties.outputs}
                    title="Outputs"
                  />
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
              <Button
                color="error"
                variant="outlined"
                size="small"
                startIcon={<FiTrash2 />}
                onClick={() => setDeleteConfirmOpen(true)}
                disabled={!item.canDelete}
              >
                Delete
              </Button>
            </div>
          </Box>
          <Dialog
            open={deleteConfirmOpen}
            onClose={() => setDeleteConfirmOpen(false)}
          >
            <DialogTitle>
              Confirm Delete
              <IconButton
                aria-label="close"
                onClick={() => setDeleteConfirmOpen(false)}
                sx={{
                  position: "absolute",
                  right: 8,
                  top: 8,
                  color: (theme) => theme.palette.grey[500],
                }}
              >
                <FiX />
              </IconButton>
            </DialogTitle>
            <DialogContent>
              <Typography>
                Are you sure you want to delete {item.name}?
              </Typography>
            </DialogContent>
            <DialogActions>
              <Button
                color="error"
                variant="outlined"
                size="small"
                startIcon={<FiTrash2 />}
                onClick={() => {
                  setDeleteConfirmOpen(false);
                  clearFocus();
                  deleteBlock(item, "spawner", {
                    name: "",
                    value: null,
                    accepts: [],
                    isSpawner: true,
                  });
                }}
                disabled={!item.canDelete}
              >
                Delete
              </Button>
            </DialogActions>
          </Dialog>
        </>
      )}

      {/* </Layer> */}
    </Drawer>
  );
};
