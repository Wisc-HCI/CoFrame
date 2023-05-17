import React, { memo, useState } from "react";
// import { Collapsible, Box } from "grommet";
import { ExpandCarrot } from "./ExpandCarrot";
import { IconButton, Card, CardHeader, Collapse as MuiCollapse, Stack } from "@mui/material";

export const Collapse = memo(({
  disabled,
  defaultOpen,
  extra,
  style = {},
  header,
  children,
  contentStyle = {},
  backgroundColor = "#303030",
  spacing = 1
}) => {

  const [collapsed, setCollapsed] = useState(!defaultOpen);
  const open = !collapsed && !disabled;

  return (
    <Card
      raised
      // variant='outlined'
      background={backgroundColor}
      sx={{ padding: '0px 5px 5px 5px', ...style }}
    >
      <CardHeader
        title={header}
        titleTypographyProps={{ variant: "subtitle1" }}
        action={
          <Stack direction='row' spacing={0.5} alignItems='center'>
            {extra}
            <IconButton color={open?'primary':'quiet'} sx={{width:31,height:31}} disabled={disabled} onClick={() => setCollapsed(!collapsed)}>
              <ExpandCarrot expanded={open} disabled={disabled} />
            </IconButton>
          </Stack>
        }
        style={{alignItems:'center'}}
      />
      <MuiCollapse in={open}>
        <Stack
          spacing={spacing}
          sx={{
            backgroundColor: "rgba(0,0,0,0.6)",
            borderRadius: 2,
            color: "white",
            padding: '5px',
            ...contentStyle
          }}
        >
          {children}
        </Stack>
      </MuiCollapse>
    </Card>
  )
})