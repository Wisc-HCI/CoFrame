import React, { useCallback, useState } from "react";
import useStore from "../../stores/Store";
import { Text } from "grommet";
import { ExternalBlock, referenceTemplateFromSpec } from "simple-vp";
import shallow from "zustand/shallow";
import { stringEquality } from "../../helpers/performance";
import {Collapse} from "../Elements/Collapse";

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

  return (
    <Collapse
        defaultOpen
        header={title}
      >
        {targets.length > 0 ? (
            targets.map((target,i) => {
              const data = target[0].dataType === 'reference' ? target[0] : referenceTemplateFromSpec(
                target[0].type,
                target[0],
                target[1]
              );
              return (
                <ExternalBlock
                  key={`${i}-${target.id}`}
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
            <Text key='no-items' alignSelf="center">No Associated Items</Text>
          )}
      </Collapse>
  );
};
