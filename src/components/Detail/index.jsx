import React, { memo, useState } from "react";
import PositionRotationTF from "./PositionRotationTF";
import { GizmoDetail } from "./GizmoDetail";
import useStore from "../../stores/Store";
import { shallow } from "zustand/shallow";
import { FiX, FiSquare, FiTrash2, FiDelete } from "react-icons/fi";
import { NumberInput } from "../Elements/NumberInput";
import { DETAIL_TYPES, STATUS } from "../../stores/Constants";
import JointGripperInput from "./JointGripperInput";
import LocationWaypointDetail from "./LocationWaypointDetail";
import { ScrollRegion } from "../Elements/ScrollRegion";
import {
  Stack,
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
  Avatar,
  Card,
  CardHeader,
} from "@mui/material";
import { BackRefSection } from "./BackRefSection";
import { ForwardRefSection } from "./ForwardRefSection";
import { DocSection } from "./DocSection";
import {
  PoseConfigurator,
  SingleRobotConfigurator,
} from "../Elements/RobotStateConfigurator";

export const Detail = memo((_) => {
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
  const setCustomMoveHook = useStore(
    (state) => state.setCustomMoveHook,
    shallow
  );
  const setCaptureFocus = useStore((state) => state.setCaptureFocus, shallow);

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
  const open = item !== null && item !== undefined;

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
      open={open}
      hideBackdrop
      onClose={() => {
        console.log("onClose");
        setCustomMoveHook(null);
        setCaptureFocus(false);
      }}
      // elevation={16}
    >
      {open && (
        <>
          <Stack
            direction="row"
            style={{
              align: "center",
              justifyContent: "space-between",
              backgroundColor: objectColor,
              padding: 1,
            }}
            spacing={0.5}
          >
            <Stack direction="row" spacing={0.25}>
              <Avatar
                variant="rounded"
                style={{
                  backgroundColor: "#22222299",
                  color: "white",
                  boxShadow: "0px 0px 1px 1px grey",
                  margin: 1,
                  height: 38,
                  width: 38,
                }}
              >
                <Icon />
              </Avatar>

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
            </Stack>

            <IconButton
              onClick={() => {
                clearFocus();
                setCustomMoveHook(null);
                setCaptureFocus(false);
              }}
            >
              <FiX />
            </IconButton>
          </Stack>

          {item.properties.status === STATUS.PENDING && (
            <LinearProgress variant="indeterminate" color="primaryColor" />
          )}

          <Stack
            style={{
              display: "flex",
              overflow: "auto",
              padding: 6,
            }}
            spacing={1}
            // border={{ color: "black", size: "xxsmall" }}
          >
            <ScrollRegion vertical>
              <Stack spacing={0.5} style={{ width: "355px", paddingTop: 7 }}>
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

                <DocSection data={item} />

                {item.properties.processTime !== undefined && (
                  <Card
                    raised
                    // variant='outlined'
                    background="#303030"
                    sx={{ padding: "0px 5px 5px 5px" }}
                  >
                    <CardHeader
                      title="Time"
                      titleTypographyProps={{ variant: "subtitle1" }}
                    />
                    <NumberInput
                      value={item.properties.processTime / 1000}
                      min={0}
                      max={Infinity}
                      onChange={(value) =>
                        updateItemSimpleProperty(
                          item.id,
                          "processTime",
                          value * 1000
                        )
                      }
                      step={0.1}
                      disabled={!item.canDelete}
                      suffix="sec"
                      style={{ width: 345 }}
                    />
                  </Card>
                )}

                {item.type === "robotAgentType" && (
                  // <>
                  //   <JointGripperInput robotID={item.id} isGripper={false} />
                  // </>
                  <SingleRobotConfigurator robot={item.id} />
                )}

                {!["waypointType", "locationType"].includes(item.type) &&
                  item.properties.position !== undefined &&
                  item.properties.rotation !== undefined && (
                    <PositionRotationTF
                      itemID={item.id}
                      disabled={!item.canEdit}
                      position={item.properties.position}
                      rotation={item.properties.rotation}
                    />
                  )}

                {(item.type === "locationType" ||
                  item.type === "waypointType") && (
                  <PoseConfigurator pose={item} />
                )}

                {(item.type === "machineType" || item.type === "toolType") && (
                  <BackRefSection
                    title="Processes"
                    reference={item.id}
                    targetType="processType"
                    targetField="gizmo"
                  />
                )}

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

                {item.properties.gizmo && (
                  <>
                    <GizmoDetail gizmoId={item.properties.gizmo} />
                  </>
                )}
              </Stack>
            </ScrollRegion>
          </Stack>
          <div style={{ display: "flex", flex: 1 }} />
          <Stack
            direction="row"
            style={{
              padding: 5,
              justifyContent: "center",
              alignContent: "center",
              borderTop: "1px solid #333333",
              backgroundColor: "#000",
            }}
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
          </Stack>
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
});
