import React, { memo, useState } from "react";
// import { Collapsible, Box } from "grommet";
import { ExpandCarrot } from "./ExpandCarrot";
import { IconButton, Card, CardHeader, Collapse, Box, Stack } from "@mui/material";

// export default function CollapseOld({
//   defaultOpen,
//   extra,
//   style = {},
//   header,
//   children,
//   contentStyle = {},
//   backgroundColor = 'rgba(0,0,0,0.6)',
//   borderWidth = 2,
//   internalPaddingWidth = 10,
// }) {
//   const [open, setOpen] = useState(defaultOpen);

//   return (
//     <Box
//       background="rgba(100,100,100,0.3)"
//       direction="column"
//       style={{
//         padding:borderWidth,
//         borderRadius: 3,
//         ...style,
//       }}
//     >
//       {header && (
//         <Box
//           direction="row"
//           flex
//           onClick={() => setOpen(!open)}
//           focusIndicator={false}
//           style={{ paddingBottom: borderWidth }}
//           justify="between"
//           align="center"
//         >
//           {header}
//           <Box
//             direction="row"
//             onClick={e=>e.stopPropagation()}
//             focusIndicator={false}
//             justify="end"
//             align="center"
//             pad={{ right: "small" }}
//           >
//             {extra}
//             <IconButton onClick={() => setOpen(!open)} size='small'>
//               <ExpandCarrot expanded={open}  />
//             </IconButton>
            
//           </Box>
//         </Box>
//       )}
//       <Collapsible open={open}>
//         <Box
//           style={{
//             ...contentStyle,
//             backgroundColor,
//             padding: internalPaddingWidth,
//             borderRadius: 3,
//             color: "white",
//           }}
//         >
//           {children ? (
//             children
//           ) : (
//             <Box height="xsmall" flex align="center" justify="center">
//               No Data
//             </Box>
//           )}
//         </Box>
//       </Collapsible>
//     </Box>
//   );
// }

const CollapseCard = memo(({
  defaultOpen,
  extra,
  style = {},
  header,
  children,
  contentStyle = {},
  backgroundColor = "#303030",
  borderWidth = 2,
  internalPaddingWidth = 10,
}) => {

  const [collapsed, setCollapsed] = useState(!defaultOpen);

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
          <Stack direction='row' spacing={0.5}>
            {extra}
            <IconButton onClick={() => setCollapsed(!collapsed)}>
              <ExpandCarrot expanded={!collapsed} />
            </IconButton>
          </Stack>
          
        }
      />
      <Collapse in={!collapsed}>
        <Stack
          spacing={1}
          sx={{
            backgroundColor: "rgba(0,0,0,0.6)",
            borderRadius: 3,
            color: "white",
            padding: '5px',
            ...contentStyle
          }}
        >
          {children}
        </Stack>
      </Collapse>
    </Card>
  )
})

export default CollapseCard