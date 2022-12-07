import React, { useCallback, useState } from "react";
import useStore from "../../stores/Store";
import { ExternalBlock, referenceTemplateFromSpec } from "simple-vp";
import shallow from "zustand/shallow";
import { stringEquality } from "../../helpers/performance";
import { Collapse } from "../Elements/Collapse";
import { Typography } from "@mui/material";

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

  return (
    <Collapse
        defaultOpen
        header={title}
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
            <Typography style={{alignSelf:"center"}}>No Associated Items</Typography>
          )}
      </Collapse>
  );
};
