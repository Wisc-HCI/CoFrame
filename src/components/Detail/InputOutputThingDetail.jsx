import React, { useCallback } from 'react';

import useStore from '../../stores/Store';

import { Toggle } from '../Toggle';
import { TextArea, Text, Box, TextInput, Accordion, AccordionPanel } from 'grommet';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';


export const InputOutputThingDetail = ({ uuid }) => {
    const input = {
        "type": "thingType",
        "name": "Knife",
        "id": "thingType-98892d7e1c1911ecbe2600155d1a70a2",
        "canEdit": true,
        "canDelete": true,
        "properties": {
            "description": "",
            "mesh": "package://evd_ros_tasks/description/markers/knife.stl",
            "safe": false,
            "weight": 0
        },
        "dataType": 0,
        "selected": false,
        "editing": false
    }

    const process = {
        "type": "processType",
        "id": "process-b3d277ad-050f-44e1-b478-75fc33656f05",
        "name": "Print Handles",
        "canDelete": true,
        "canEdit": true,
        "properties": {
            "description": "Print handles on the 3d Printer",
            "processTime": 60000,
            "machine": "machine-ac5a063a-9ff9-4c0a-8fa6-81d3983f4a10",
            "inputs": [],
            "outputs": [
                {
                    "thingType": "thingType-98892a7c1c1911ecbe2600155d1a70a2",
                    "relativeTo": "machine-ac5a063a-9ff9-4c0a-8fa6-81d3983f4a10",
                    "position": {
                        "x": -0.2,
                        "y": 0.28,
                        "z": 0.074
                    },
                    "orientation": {
                        "x": 0.707,
                        "y": 0,
                        "z": 0,
                        "w": 0.707
                    }
                },
                {
                    "thingType": "thingType-98892b3a1c1911ecbe2600155d1a70a2",
                    "relativeTo": "machine-ac5a063a-9ff9-4c0a-8fa6-81d3983f4a10",
                    "position": {
                        "x": -0.3,
                        "y": 0.28,
                        "z": 0.078
                    },
                    "orientation": {
                        "x": 0.707,
                        "y": 0,
                        "z": 0,
                        "w": 0.707
                    }
                }
            ]
        },
        "dataType": 0,
        "selected": false,
        "editing": false

    }


    return (
        <>
            <br />

            <Box round="xsmall" pad="small" background="#303030" >
                <b style={{ color: 'rgba(255, 255, 255, 0.85)', paddingBottom: '2%' }} >Description : </b>
                <div>
                    <TextArea
                        value={input.properties.description}
                        disabled={!input.canDelete}
                    />
                </div>
            </Box>

            <br />


            <Box
                round="xsmall"
                background="#303030"
                pad="small" >
                <Box direction='column'>
                    <b style={{ color: 'rgba(255, 255, 255, 0.85)', paddingBottom: '2%' }} >Properties : </b>
                    <div style={{ paddingTop: '4%' }}>
                        <Box background="grey" round="small" pad="xsmall">
                            <div style={{ display: 'flex', flexDirection: 'row', justifyContent: "center", justifyItems: "center" }}>
                                <Text style={{ paddingRight: '42%' }}>Placement :</Text>
                            </div>
                            <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'flex-start', paddingLeft: '30%' }}>

                                <PositionInput />
                                <OrientationInput />
                            </div>

                            <br />
                        </Box>
                    </div>


                </Box>
            </Box>




        </>

    )
}
