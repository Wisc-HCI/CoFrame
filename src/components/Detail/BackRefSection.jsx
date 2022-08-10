import React, { useCallback, useState } from "react";
import useStore from "../../stores/Store";
import { Box, Collapsible, Text } from "grommet";
import { ExternalBlock, referenceTemplateFromSpec } from "simple-vp";
import shallow from "zustand/shallow";
import { stringEquality } from "../../helpers/performance";
import { Card, CardHeader, IconButton, Collapse } from "@mui/material";
import { ExpandCarrot } from "../Elements/ExpandCarrot";

export const BackRefSection = ({
  title,
  reference,
  targetType,
  targetField,
}) => {
  const typeInfo = useStore(
    useCallback(
      (state) => state.programSpec.objectTypes[targetType],
      [targetType]
    ),
    shallow
  );
  const addFocusItem = useStore((state) => state.addFocusItem, shallow);
  const targets = useStore((state) => {
    return Object.values(state.programData).filter(
      (value) =>
        value &&
        value.properties &&
        value.type === targetType &&
        value.properties[targetField] === reference
    );
  }, stringEquality);

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
              const targetRef = referenceTemplateFromSpec(
                targetType,
                target,
                typeInfo
              );
              return (
                <ExternalBlock
                  key={target.id}
                  store={useStore}
                  data={targetRef}
                  highlightColor={"#333333"}
                  onClick={() => {
                    addFocusItem(target.id, true);
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
