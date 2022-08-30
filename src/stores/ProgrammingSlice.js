import { DATA_TYPES, TYPES } from "simple-vp";
import { remove, pickBy } from "lodash";
import { instanceTemplateFromSpec } from "simple-vp";
import { generateUuid } from "./generateUuid";
import { STATUS } from "./Constants";
// import { FiUnderline } from "react-icons/fi";

// Credit: https://www.npmjs.com/package/lodash-move
export function move(array, moveIndex, toIndex) {
  /* #move - Moves an array item from one position in an array to another.
     Note: This is a pure function so a new array will be returned, instead
     of altering the array argument.
    Arguments:
    1. array     (String) : Array in which to move an item.         (required)
    2. moveIndex (Object) : The index of the item to move.          (required)
    3. toIndex   (Object) : The index to move item at moveIndex to. (required)
  */
  const item = array[moveIndex];
  const length = array.length;
  const diff = moveIndex - toIndex;

  if (diff > 0) {
    // move left
    return [
      ...array.slice(0, toIndex),
      item,
      ...array.slice(toIndex, moveIndex),
      ...array.slice(moveIndex + 1, length),
    ];
  } else if (diff < 0) {
    // move right
    const targetIndex = toIndex;
    return [
      ...array.slice(0, moveIndex),
      ...array.slice(moveIndex + 1, targetIndex),
      item,
      ...array.slice(targetIndex, length),
    ];
  }
  return array;
}

export function deleteFromChildren(state, idsToDelete, parentData) {
  // Corner case for call blocks (don't look at parent's information)
  if (parentData && parentData.dataType === DATA_TYPES.CALL) {
    Object.keys(parentData.properties)?.forEach((propName) => {
      if (
        idsToDelete.includes(
          state.programData[parentData.properties[propName]]?.ref
        )
      ) {
        delete state.programData[parentData.properties[propName]];
        state.programData[parentData.id].properties[propName] = null;
        state.programData[parentData.id].properties.status = STATUS.PENDING;
      }
    });
    for (let i = 0; i < idsToDelete.length; i++) {
      delete state.programData[parentData.id].properties[idsToDelete[i]];
    }
  } else if (parentData) {
    // Clear children and properties (if applicable)
    if (state.programSpec.objectTypes[parentData.type]?.properties) {
      Object.keys(
        state.programSpec.objectTypes[parentData.type].properties
      ).forEach((propName) => {
        if (propName) {
          const property =
            state.programSpec.objectTypes[parentData.type].properties[propName];

          // Clearing child fields/references
          if (property && (property.type || property.type === TYPES.OBJECT)) {
            // Ignore SIMPLE types.
          } else if (property && property.isList) {
            parentData.properties[propName].forEach((child) => {
              state = deleteFromChildren(
                state,
                idsToDelete,
                state.programData[child]
              );
            });
            idsToDelete.forEach((idToDelete) => {
              const newList = state.programData[parentData.id].properties[
                propName
              ].filter((field) => state.programData[field]?.ref !== idToDelete);
              if (
                newList.length !==
                state.programData[parentData.id].properties[propName]
              ) {
                state.programData[parentData.id].properties.status =
                  STATUS.PENDING;
              }
              state.programData[parentData.id].properties[propName] = newList;
            });
          } else if (
            property &&
            parentData.properties[propName] &&
            idsToDelete.includes(
              state.programData[parentData.properties[propName]]?.ref
            )
          ) {
            // Delete Reference to Child
            delete state.programData[parentData.properties[propName]];
            // entry.properties[propName] = null;
            state.programData[parentData.id].properties[propName] = null;
            state.programData[parentData.id].properties.status = STATUS.PENDING;
          }
        }
      });
    }
  }

  return state;
}

export function deleteFromProgram(state, idsToDelete) {
  const searches = pickBy(
    state.programData,
    (entry) => entry.dataType === DATA_TYPES.INSTANCE
  );
  // Search through all instances for occurances of the ids we're deleting
  Object.keys(searches).forEach((entry) => {
    if (state.programData[entry]) {
      state = deleteFromChildren(state, idsToDelete, state.programData[entry]);
    }
  });

  return state;
}

export function deleteSelfBlock(state, data, parentId, fieldInfo) {
  if (data.typeSpec?.type === TYPES.FUNCTION) {
    // Find all references to the function
    const callReferences = pickBy(
      state.programData,
      (entry) =>
        entry.dataType === DATA_TYPES.CALL && entry.refData.id === data.id
    );
    const callIds = Object.keys(callReferences);

    // Delete the children from the function calls
    callIds.forEach((cID) => {
      state = deleteChildren(
        state,
        state.programData[cID],
        parentId,
        fieldInfo
      );
    });

    // Clear function arguments if function
    if (data.arguments) {
      data.arguments.forEach((argumentId) => {
        delete state.programData[argumentId];
      });
    }

    // Find the parent's of the references, and remove the references from them
    Object.keys(state.programData).forEach((entryId) => {
      const entry = state.programData[entryId];
      if (state.programSpec.objectTypes[entry.type].properties) {
        // Iterate through properties
        Object.keys(
          state.programSpec.objectTypes[entry.type].properties
        ).forEach((propName) => {
          if (propName) {
            const property =
              state.programSpec.objectTypes[entry.type].properties[propName];

            if (property && (property.type || property.type === TYPES.OBJECT)) {
              // Ignore SIMPLE types.
            } else if (property && property.isList) {
              // Iterate through property list and remove all applicable references
              for (let i = 0; i < callIds.length; i++) {
                if (entry && entry.properies && entry.properties[propName]?.includes(callIds[i])) {
                  let removed_elems = remove(
                    state.programData[entryId].properties[propName],
                    (field) => field === callIds[i]
                  );
                  // Only update if elements were removed
                  if (removed_elems.length > 0) {
                    state.programData[entryId].properties.status =
                      STATUS.PENDING;
                  }
                }
              }
            } else if (property && entry && entry.properties && entry.properties[propName]) {
              // Delete reference from property
              if (callIds.includes(entry.properties[propName])) {
                entry.properties[propName] = null;
                state.programData[entryId].properties[propName] = null;
                state.programData[entryId].properties.status = STATUS.PENDING;
              }
            }
          }
        });
      }
    });

    // Delete the reference and any children
    callIds.forEach((reference) => {
      state = deleteChildren(state, state.programData[reference]);
      delete state.programData[reference];
    });
  } else if (fieldInfo?.isSpawner) {
    if (parentId === "spawner") {
      // Drawer deletion
      if (data.dataType === DATA_TYPES.INSTANCE) {
        state = deleteFromProgram(state, [data.id]);
        delete state.programData[data.id];
      } else {
        state = deleteFromProgram(state, [data.ref]);
        delete state.programData[data.ref];
      }
    } else {
      // Argument deletion
      state = deleteFromChildren(
        state,
        [data.ref],
        state.programData[parentId]
      );

      // Remove argument from function
      if (state.programData[parentId]?.arguments) {
        remove(
          state.programData[parentId]?.arguments,
          (field) => field === data.ref
        );
      }
    }
  }

  // Remove self from state
  delete state.programData[data.id];

  return state;
}

export function deleteChildren(state, data, parentId, fieldInfo) {
  // Clear children and properties (if applicable)
  if (data.dataType === DATA_TYPES.CALL) {
    state.programData[data.ref].arguments.forEach((argument) => {
      if (
        data.properties[argument] &&
        state.programData[data.properties[argument]]
      ) {
        state = deleteSelfBlock(
          state,
          state.programData[data.properties[argument]],
          parentId,
          fieldInfo
        );
      }
    });
  } else if (
    data.dataType !== DATA_TYPES.REFERENCE &&
    parentId !== 'spawner' &&
    state.programSpec.objectTypes[data.type].properties
  ) {
    Object.keys(state.programSpec.objectTypes[data.type].properties).forEach(
      (propName) => {
        if (propName) {
          const property =
            state.programSpec.objectTypes[data.type].properties[propName];

          // Clearing child fields/references
          if (property && (property.type || property.type === TYPES.OBJECT)) {
            // Ignore SIMPLE types.
          } else if (property && property.isList) {
            // Iterate over list and remove each entry (probably recursively)
            if (data.properties[propName]) {
              data.properties[propName].forEach((child) => {
                // Recursively delete children
                state = deleteChildren(
                  state,
                  state.programData[child],
                  parentId,
                  fieldInfo
                );
                state = deleteSelfBlock(
                  state,
                  state.programData[child],
                  parentId,
                  fieldInfo
                );
              });
            }
          } else if (property && data.properties[propName]) {
            // Delete Reference to Child
            state = deleteChildren(
              state,
              state.programData[data.properties[propName]],
              parentId,
              fieldInfo
            );
            state = deleteSelfBlock(
              state,
              state.programData[data.properties[propName]],
              parentId,
              fieldInfo
            );
          }
        }
      }
    );
  }

  return state;
}

export const ProgrammingSliceOverride = (set, get) => ({
  forceRefreshBlock: (id) =>
    set((state) => {
      state.programData[id].properties.status = STATUS.PENDING;
    }),
  transferBlock: (data, sourceInfo, destInfo) => {
    set((state) => {
      let newSpawn = false;
      let id = data.id;

      if (!state.programData[data.id]) {
        // Clone the data with a new id
        id = generateUuid(data.type);
        state.programData[id] = { ...data, id };
        newSpawn = true;
      }

      const sourceIsList = sourceInfo.fieldInfo?.isList;
      const destIsList = destInfo.fieldInfo.isList;

      // If both source and dest are the same list, handle this specially
      if (
        destIsList &&
        sourceIsList &&
        sourceInfo.parentId === destInfo.parentId
      ) {
        state.programData[destInfo.parentId].properties[
          destInfo.fieldInfo.value
        ] = move(
          state.programData[destInfo.parentId].properties[
            destInfo.fieldInfo.value
          ],
          sourceInfo.idx,
          destInfo.idx
        );
      } else {
        // Place the value in its new location
        if (destIsList) {
          state.programData[destInfo.parentId].properties[
            destInfo.fieldInfo.value
          ].splice(destInfo.idx, 0, id);
        } else {
          state.programData[destInfo.parentId].properties[
            destInfo.fieldInfo.value
          ] = id;
        }
        // If existing, remove from the previous location
        if (
          !newSpawn &&
          sourceInfo.parentId === destInfo.parentId &&
          sourceInfo.fieldInfo === destInfo.fieldInfo
        ) {
          // ignore if dropped in the source
        } else if (!newSpawn && sourceIsList) {
          // Insert at the right location
          state.programData[sourceInfo.parentId].properties[
            destInfo.fieldInfo.value
          ].splice(sourceInfo.idx, 1);
        } else if (!newSpawn && !sourceIsList) {
          console.log("removing from previous by setting to null");
          state.programData[sourceInfo.parentId].properties[
            sourceInfo.fieldInfo.value
          ] = null;
        }
      }

      if (!newSpawn) {
        state.programData[sourceInfo.parentId].properties.status =
          STATUS.PENDING;
        // console.log({editItem:state.programData[id],itemProps:state.programData[id].properties,id})
        // state.programData[id].properties.status = STATUS.PENDING;
      }
      state.programData[destInfo.parentId].properties.status = STATUS.PENDING;
    });
  },
  deleteBlock: (data, parentId, fieldInfo) => {
    set((state) => {
      // Delete block's children and parameters
      state = deleteChildren(state, data, parentId, fieldInfo);

      // Delete current block
      state = deleteSelfBlock(state, data, parentId, fieldInfo);

      // Clear parent properties
      if (parentId !== "spawner") {
        if (parentId && fieldInfo && !fieldInfo.isList) {
          // Clear parent's field value (to null)
          state.programData[parentId].properties[fieldInfo.value] = null;
        } else if (parentId && fieldInfo && fieldInfo.isList) {
          // Erase self from the parent's list
          remove(
            state.programData[parentId].properties[fieldInfo.value],
            (entry) => entry === data.id
          );
        }
      }

      if (state.programData[parentId] !== undefined) {
        state.programData[parentId].properties.status = STATUS.PENDING;
      }
    });
  },
  createPlacedBlock: (data, x, y) => {
    set((state) => {
      let id = data.id;

      if (!state.programData[data.id]) {
        // Clone the data with a new id
        id = generateUuid(data.type);
        state.programData[id] = { ...data, id };
      }

      state.programData[id].position = { x, y };
    });
  },
  addArgument: (parentFunctionId, argumentType) => {
    set((state) => {
      const id = generateUuid(argumentType);
      const template = {
        ...instanceTemplateFromSpec(
          argumentType,
          state.programSpec.objectTypes[argumentType],
          true
        ),
        id,
        dataType: DATA_TYPES.ARGUMENT,
      };
      state.programData[id] = template;
      state.programData[parentFunctionId].arguments.push(id);
      state.programData[id].properties.status = STATUS.PENDING;
      Object.values(state.programData).forEach((node) => {
        if (
          node.dataType === DATA_TYPES.CALL &&
          node.ref === parentFunctionId
        ) {
          state.programData[node.id].properties.status = STATUS.PENDING;
        }
      });
    });
  },
  updateItemSimpleProperty: (id, property, value) => {
    if (value === undefined) {
      set((state) => {
        state.programData[id].properties.initialJointState = property;
        state.programData[id].properties.status = STATUS.PENDING;
      });
    } else {
      set((state) => {
        state.programData[id].properties[property] = value;
        state.programData[id].properties.status = STATUS.PENDING;
      });
    }
  },
  updateItemPositionProperty: (id, property, value) => {
    set((state) => {
      state.programData[id].properties.position[property] = value;
      state.programData[id].properties.status = STATUS.PENDING;
    });
  },
  updateItemRotationProperty: (id, value) => {
    set((state) => {
      state.programData[id].properties.rotation["w"] = value[0];
      state.programData[id].properties.rotation["x"] = value[1];
      state.programData[id].properties.rotation["y"] = value[2];
      state.programData[id].properties.rotation["z"] = value[3];
      state.programData[id].properties.status = STATUS.PENDING;
    });
  },
  updateItemDescription: (id, value) => {
    set((state) => {
      state.programData[id].properties.description = value;
    });
  },
});
