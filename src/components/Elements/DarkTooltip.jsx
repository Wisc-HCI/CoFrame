import { Tooltip, alpha } from "@mui/material";
import { tooltipClasses } from '@mui/material/Tooltip';
import styled from "@emotion/styled";

export const DarkTooltip = styled(({ className, ...props }) => (
    <Tooltip {...props} arrow classes={{ popper: className }} />
  ))(({ theme }) => ({
    [`& .${tooltipClasses.arrow}`]: {
      color: alpha(theme.palette.common.black,0.9)
    },
    [`& .${tooltipClasses.tooltip}`]: {
      backgroundColor: alpha(theme.palette.common.black,0.9),
    },
  }));