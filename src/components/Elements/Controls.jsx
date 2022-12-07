import React, { useState } from "react";

// import { CaretRightFilled, PauseOutlined, RollbackOutlined, FullscreenExitOutlined, FullscreenOutlined, EllipsisOutlined } from '@ant-design/icons';
import {
  FiMinimize,
  FiMaximize,
  FiMoreHorizontal,
  FiCheckCircle,
  FiCircle
} from "react-icons/fi";
import {
  IconButton,
  ListItemIcon,
  ListItemText,
  Menu,
  MenuItem,
  Stack,
} from "@mui/material";
import useStore from "../../stores/Store";
import shallow from "zustand/shallow";

export function Controls() {
  const [
    primaryColor,
    viewMode,
    setViewMode,
    collisionsVisible,
    setCollisionsVisible,
    occupancyVisible,
    setOccupancyVisible,
    tfVisible,
    setTfVisible,
    robotPreviewVisible,
    setRobotPreviewVisible
  ] = useStore(
    (state) => [
      state.primaryColor,
      state.viewMode,
      state.setViewMode,
      state.collisionsVisible,
      state.setCollisionsVisible,
      state.occupancyVisible,
      state.setOccupancyVisible,
      state.tfVisible,
      state.setTfVisible,
      state.robotPreviewVisible,
      state.setRobotPreviewVisible
    ],
    shallow
  );

  const [anchorEl, setAnchorEl] = useState(null);
  const open = Boolean(anchorEl);
  const handleClick = (event) => {
    setAnchorEl(event.currentTarget);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  return (
    <Stack direction='row' gap={1} alignItems='center'>
      <IconButton
        size="small"
        onClick={() => setViewMode(viewMode === "default" ? "sim" : "default")}
      >
        {viewMode === "default" ? <FiMaximize /> : <FiMinimize />}
      </IconButton>
      <IconButton
        size="small"
        id="basic-button"
        aria-controls={open ? "basic-menu" : undefined}
        aria-haspopup="true"
        aria-expanded={open ? "true" : undefined}
        onClick={handleClick}
      >
        <FiMoreHorizontal />
      </IconButton>
      <Menu
        id="basic-menu"
        anchorEl={anchorEl}
        open={open}
        onClose={handleClose}
        MenuListProps={{
          "aria-labelledby": "basic-button",
        }}
      >
        <MenuItem onClick={()=>setCollisionsVisible(!collisionsVisible)}>
          <ListItemIcon>
            {collisionsVisible ? <FiCheckCircle /> : <FiCircle/>}
          </ListItemIcon>
          <ListItemText>Show Collisions</ListItemText>
        </MenuItem>
        <MenuItem onClick={()=>setOccupancyVisible(!occupancyVisible)}>
          <ListItemIcon>
            {occupancyVisible ? <FiCheckCircle /> : <FiCircle/>}
          </ListItemIcon>
          <ListItemText>Show Occupancy</ListItemText>
        </MenuItem>
        <MenuItem onClick={()=>setTfVisible(!tfVisible)}>
          <ListItemIcon>
            {tfVisible ? <FiCheckCircle /> : <FiCircle/>}
          </ListItemIcon>
          <ListItemText>Show TFs</ListItemText>
        </MenuItem>
        <MenuItem onClick={()=>setRobotPreviewVisible(!robotPreviewVisible)}>
          <ListItemIcon>
            {robotPreviewVisible ? <FiCheckCircle /> : <FiCircle/>}
          </ListItemIcon>
          <ListItemText>Robot Preview for Trajectories</ListItemText>
        </MenuItem>
      </Menu>
    </Stack>
  );
}

