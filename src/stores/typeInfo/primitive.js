import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { PrimitiveIconStyled } from "./icons";
import { merge } from 'lodash';

const basicPrimitiveData = {
    delayType: {
        name: 'Delay',
        type: TYPES.OBJECT,
        instanceBlock: {
          hideNewPrefix: true,
          onCanvas: false,
          color: "#629e6c",
          icon: PrimitiveIconStyled,
          extras: [
            EXTRA_TYPES.LOCKED_INDICATOR,
            {
              icon: FiMoreHorizontal,
              type: EXTRA_TYPES.DROPDOWN,
              contents: [
                EXTRA_TYPES.DELETE_BUTTON,
                EXTRA_TYPES.DEBUG_TOGGLE
              ]
            }
          ]
        },
        referenceBlock: null,
        properties: {
          description: {
            name: 'Description',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: "",
            isList: false,
            fullWidth: true
          },
          duration: {
            name: 'Duration',
            type: SIMPLE_PROPERTY_TYPES.NUMBER,
            default: 1,
            min: 0,
            max: 5
          }
        }
    }
}

// export const 