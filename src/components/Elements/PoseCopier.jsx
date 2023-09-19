import {
  FormControl,
  Select,
  InputLabel,
  MenuItem,
  Button,
  Menu,
} from "@mui/material";
import useStore from "../../stores/Store";
import { pickBy } from "lodash";
import { shallow } from "zustand/shallow";
import { DATA_TYPES } from "open-vp";
import { useState } from "react";

export const PoseCopier = ({ disabled, onSelect = () => {} }) => {
  const [anchorEl, setAnchorEl] = useState(null);
  const open = Boolean(anchorEl);
  const handleClick = (event) => {
    setAnchorEl(event.currentTarget);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  const poses = useStore(
    (state) =>
      pickBy(
        state.programData,
        (value) =>
          ["waypointType", "locationType"].includes(value.type) &&
          value.dataType === DATA_TYPES.INSTANCE
      ),
    shallow
  );

  return (
    <div>
      <Button
        variant='outlined'
        color="primary"
        fullWidth
        id="basic-button"
        aria-controls={open ? "basic-menu" : undefined}
        aria-haspopup="true"
        aria-expanded={open ? "true" : undefined}
        onClick={handleClick}
      >
        Copy From Pose
      </Button>
      <Menu
        id="copy-menu"
        anchorEl={anchorEl}
        open={open}
        onClose={handleClose}
        MenuListProps={{
          "aria-labelledby": "basic-button",
        }}
      >
        {Object.keys(poses).map((key) => (
          <MenuItem
            key={key}
            value={key}
            onClick={(e) => {
              onSelect(poses[key]);
              handleClose();
            }}
          >
            {poses[key].name}
          </MenuItem>
        ))}
      </Menu>
    </div>
  );
};
