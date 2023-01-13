import React, { useState } from "react";

import {
  FiMinimize,
  FiMaximize,
  FiMoreHorizontal,
  FiCheckCircle,
  FiCircle,
  FiPause,
  FiPlay,
} from "react-icons/fi";
import {
  IconButton,
  ListItemIcon,
  ListItemText,
  Menu,
  MenuItem,
  Stack,
  Slider,
  useTheme,
} from "@mui/material";
import useStore from "../../stores/Store";
import useCompiledStore from "../../stores/CompiledStore";
import shallow from "zustand/shallow";
import { useTime } from "../useTime";
import { STATUS, STEP_TYPE, TIMELINE_TYPES } from "../../stores/Constants";
import { useCallback } from "react";

export function Controls() {
  const [
    viewMode,
    setViewMode,
    collisionsVisible,
    setCollisionsVisible,
    occupancyVisible,
    setOccupancyVisible,
    tfVisible,
    setTfVisible,
    robotPreviewVisible,
    setRobotPreviewVisible,
  ] = useStore(
    (state) => [
      state.viewMode,
      state.setViewMode,
      state.collisionsVisible,
      state.setCollisionsVisible,
      state.occupancyVisible,
      state.setOccupancyVisible,
      state.tfVisible,
      state.setTfVisible,
      state.robotPreviewVisible,
      state.setRobotPreviewVisible,
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

  const focusItem = useStore(
    (state) => {
      let focused = null;
      state.focus.some((focusItem) => {
        const status = state.programData[focusItem]?.properties.status;
        if (TIMELINE_TYPES.includes(state.programData[focusItem]?.type) && (status === STATUS.VALID || status === STATUS.WARN)) {
          focused = focusItem;
          return true
        }
        return false
      });
      return focused
    },
    shallow
  );

  console.log(focusItem)

  return (
    <Stack direction="row" gap={1} alignItems="center">
      {focusItem && <MediaControls item={focusItem}/>}
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
        <MenuItem onClick={() => setCollisionsVisible(!collisionsVisible)}>
          <ListItemIcon>
            {collisionsVisible ? <FiCheckCircle /> : <FiCircle />}
          </ListItemIcon>
          <ListItemText>Show Collisions</ListItemText>
        </MenuItem>
        <MenuItem onClick={() => setOccupancyVisible(!occupancyVisible)}>
          <ListItemIcon>
            {occupancyVisible ? <FiCheckCircle /> : <FiCircle />}
          </ListItemIcon>
          <ListItemText>Show Occupancy</ListItemText>
        </MenuItem>
        <MenuItem onClick={() => setTfVisible(!tfVisible)}>
          <ListItemIcon>
            {tfVisible ? <FiCheckCircle /> : <FiCircle />}
          </ListItemIcon>
          <ListItemText>Show TFs</ListItemText>
        </MenuItem>
        <MenuItem onClick={() => setRobotPreviewVisible(!robotPreviewVisible)}>
          <ListItemIcon>
            {robotPreviewVisible ? <FiCheckCircle /> : <FiCircle />}
          </ListItemIcon>
          <ListItemText>Robot Preview for Trajectories</ListItemText>
        </MenuItem>
      </Menu>
    </Stack>
  );
}

const MediaControls = ({item}) => {
  const theme = useTheme();
  const playing = useStore((state) => state.playing, shallow);
  const play = useStore((state) => state.play, shallow);
  const pause = useStore((state) => state.pause, shallow);
  const reset = useStore((state) => state.reset, shallow);

  const totalTime = useCompiledStore(useCallback(state=>{
    let steps = [];
    let time = 0.0
    if (state[item] && Object.keys(state[item]).length === 1) {
      steps = state[item][Object.keys(state[item])[0]]?.steps;
    }
    steps.reverse();
    steps.some((step)=>{
      if (step.type === STEP_TYPE.ACTION_END) {
        time = step.time
        return true
      } 
      return false
    })
    return time
  },[item]),shallow)

  const onTogglePlayPause = () => {
    if (playing) {
      pause();
    } else {
      play(1);
    }
  };

  // const time = useStore(state=>state.clock._elapsed_time,shallow)
  const time = useTime(totalTime);

  return totalTime > 0 ? (
    <>
      <Slider 
        size="small" 
        value={time/1000} min={0} max={totalTime/1000} step={0.1}
        onChange={(_,value)=>{
          console.log(value)
          reset(value);
          pause();
        }}
        sx={{
          width: 100,
          color: theme.palette.mode === 'dark' ? '#fff' : 'rgba(0,0,0,0.87)',
          height: 4,
          '& .MuiSlider-thumb': {
            width: 8,
            height: 8,
            transition: '0.3s cubic-bezier(.47,1.64,.41,.8)',
            '&:before': {
              boxShadow: '0 2px 12px 0 rgba(0,0,0,0.4)',
            },
            '&:hover, &.Mui-focusVisible': {
              boxShadow: `0px 0px 0px 8px ${
                theme.palette.mode === 'dark'
                  ? 'rgb(255 255 255 / 16%)'
                  : 'rgb(0 0 0 / 16%)'
              }`,
            },
            '&.Mui-active': {
              width: 20,
              height: 20,
            },
          },
          '& .MuiSlider-rail': {
            opacity: 0.28,
          },
        }}
      />
      <IconButton onClick={onTogglePlayPause} size="small">
        {playing ? <FiPause /> : <FiPlay />}
      </IconButton>
    </>
  ) : null;
};
