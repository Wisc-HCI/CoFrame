import { sceneBase } from "./sceneBase";
import { SIMPLE_PROPERTY_TYPES } from "simple-vp";

export const linkType = {
    ...sceneBase,
    name: 'Link',
    properties: {
        robot: {
            name: 'Robot Ref',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: '',
            isList: false,
            fullWidth: true
        },
        collision: {
            name: 'Collision',
            type: SIMPLE_PROPERTY_TYPES.IGNORED, 
            default: '',
            isList: false,
            fullWidth: true
        }
    }
}