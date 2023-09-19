import {
    ToggleButtonGroup as MuiToggleButtonGroup
  } from "@mui/material";
  import { styled, alpha } from "@mui/material/styles";
  
//   export const MotionHandle = motion(Handle);
  
  export const ToggleButtonGroup = styled(MuiToggleButtonGroup)(({ theme }) => ({
    backgroundColor: alpha(theme.palette.background.default, 0.85),
    height: 30,
    borderRadius: theme.shape.borderRadius,
    "& .MuiToggleButtonGroup-grouped": {
      margin: theme.spacing(0.5),
      border: 0,
      height: 18,
      flex: 1,
      "&.Mui-disabled": {
        border: 0,
      },
      "&:not(:first-of-type)": {
        borderRadius: theme.shape.borderRadius * 0.66,
      },
      "&:first-of-type": {
        borderRadius: theme.shape.borderRadius * 0.66,
      },
    },
  }));
  
//   export const ToggleButtonGroup = motion(StyledToggleButtonGroup);