import React, { useCallback, useState } from "react";
import useStore from "../../stores/Store";
import { Box, Collapsible, Text } from "grommet";
import { ExternalBlock, referenceTemplateFromSpec } from "simple-vp";
import shallow from "zustand/shallow";
import { stringEquality } from "../../helpers/performance";
import { Card, CardHeader, IconButton, Collapse } from "@mui/material";
import { ExpandCarrot } from "../Elements/ExpandCarrot";

export const ForwardRefSection = ({
  title,
  references
}) => {
//   const typeInfo = useStore(
//     useCallback(
//       (state) => state.programSpec.objectTypes[targetType],
//       [targetType]
//     ),
//     shallow
//   );
  const addFocusItem = useStore((state) => state.addFocusItem, shallow);
  const targets = useStore((state) => {
    return references
        .map(reference=>state.programData[reference])
        .filter(d=>d)
        .map(d=>([d,state.programSpec.objectTypes[d.type]]))
    // return Object.values(state.programData).filter(
    //   (value) =>
    //     value &&
    //     value.properties &&
    //     value.type === targetType &&
    //     value.properties[targetField] === reference
    // );
  }, stringEquality);

  console.log(targets)

  const [collapsed, setCollapsed] = useState(false);

  return (
    <Card
      raised
      // variant='outlined'
      background="#303030"
      sx={{ padding: '0px 5px 5px 5px' }}
    >
      <CardHeader
        title={title}
        titleTypographyProps={{ variant: "subtitle1" }}
        action={
          <IconButton onClick={() => setCollapsed(!collapsed)}>
            <ExpandCarrot expanded={!collapsed} />
          </IconButton>
        }
      />
      <Collapse in={!collapsed}>
        <Box
          gap="xsmall"
          pad="xsmall"
          style={{
            backgroundColor: "rgba(0,0,0,0.6)",
            borderRadius: 3,
            color: "white",
          }}
        >
          {targets.length > 0 ? (
            targets.map((target) => {
              const data = target[0].dataType === 'reference' ? target[0] : referenceTemplateFromSpec(
                target[0].type,
                target[0],
                target[1]
              );
              return (
                <ExternalBlock
                  key={target.id}
                  store={useStore}
                  data={data}
                  highlightColor={"#333333"}
                  onClick={() => {
                    addFocusItem(target[0].id, true);
                  }}
                />
              );
            })
          ) : (
            <Text alignSelf="center">No Associated Items</Text>
          )}
        </Box>
      </Collapse>
    </Card>
  );
};
