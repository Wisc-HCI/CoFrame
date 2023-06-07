import React, { memo } from "react";
import { motion } from "framer-motion";
import SvgIcon from "@mui/material/SvgIcon";
import { useTheme } from "@mui/material";

export const ExpandCarrot = memo(({ expanded, disabled, flip=false, fontSize=15 }) => {
  const theme = useTheme();

  const openPath = flip ? "M585.078,28.156L212.387,399.156L585.078,771.847" : "M770.578,215.347L399.578,586.347L26.887,213.656";
  const closedPath = "M214.078,28.156L585.078,399.156L212.387,771.847";
  const variants = {
    openDisabled: { d: openPath, stroke: theme.palette.quiet.main },
    closedDisabled: { d: closedPath, stroke: theme.palette.quiet.main },
    openEnabled: { d: openPath, stroke: theme.palette.primary.main},
    closedEnabled: { d: closedPath, stroke: theme.palette.primary.main }
  };
  
  const variant = expanded && disabled 
    ? 'openDisabled' : expanded && !disabled 
    ? 'openEnabled' : !expanded && disabled 
    ? 'closedDisabled' : 'closedEnabled';

  return (
    <SvgIcon
      sx={{ fontSize }}
      viewBox="0 0 800 800"
      style={{
        fillRule: "evenodd",
        clipRule: "evenodd",
        strokeLinecap: "round",
        strokeLinejoin: "round",
        strokeMiterlimit: 1.5,
      }}
    >
      <motion.path
        animate={variant}
        variants={variants}
        style={{
          fill: "none",
          // stroke: disabled ? theme.palette.quiet.main : theme.palette.primary.main,
          strokeOpacity: 1,
          strokeWidth: 100,
        }}
      />
    </SvgIcon>
  );
});
