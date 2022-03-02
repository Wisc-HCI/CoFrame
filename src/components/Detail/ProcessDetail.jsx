import React, { useCallback } from 'react';

import useStore from '../../stores/Store';

import { Toggle } from '../Toggle';
import { TextArea, Text, Box, TextInput, Accordion, AccordionPanel } from 'grommet';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';


export const ProcessDetail = ({ uuid }) => {

    console.log({ uuid });

    // const machine = useStore(useCallback(state => state.data.machines[uuid], [uuid]));
    // const setItemProperty = useStore(state => state.setItemProperty);

    //const { TextArea } = Input;
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
                        value={process.properties.description}
                        disabled={!process.canDelete}
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
                    <Box background="grey" round="small" pad="xsmall">
                        <div style={{ display: 'flex', flexDirection: 'row', justifyContent: "center", justifyItems: "center" }}>
                            <Text style={{ paddingRight: '3%' }}>Time :</Text>
                            <div style={{ display: 'flex', flexDirection: 'row', paddingRight: '30%' }}>
                                <TextInput style={{ maxWidth: 40 }} value={process.properties.processTime} disabled={!process.canDelete} />
                                <Text style={{ paddingLeft: "4%" }}>sec</Text>
                            </div>
                        </div>
                    </Box>

                    <div style={{ paddingTop: '4%' }}>
                        <Accordion>
                            <AccordionPanel label="Inputs :">
                                <Box round="xsmall" background="grey" direction='column'
                                    elevation="none" pad="xsmall" justify='center'>
                                    <Box round="xsmall" background="grey" direction='column'
                                        elevation="none" pad="xsmall" justify='center'
                                        hoverIndicator={true} onClick={() => { }}>
                                        <Text>Input 1</Text>
                                    </Box>
                                    <br />
                                    <Box round="xsmall" background="grey" direction='column'
                                        elevation="none" pad="xsmall" justify='center' hoverIndicator={true} onClick={() => { }}>
                                        <Text>Input 2</Text>
                                    </Box>
                                    <br />
                                    <Box round="xsmall" background="grey" direction='column'
                                        elevation="none" pad="xsmall" justify='center' hoverIndicator={true} onClick={() => { }}>
                                        <Text>Input 3</Text>
                                    </Box>
                                </Box>


                            </AccordionPanel>
                        </Accordion>
                        <Accordion>
                            <AccordionPanel label="Outputs :">
                                <Box round="xsmall" background="grey" direction='column'
                                    elevation="none" pad="xsmall" justify='center'>
                                    <Box round="xsmall" background="grey" direction='column'
                                        elevation="none" pad="xsmall" justify='center'
                                        hoverIndicator={true} onClick={() => { }}>
                                        <Text>Output 1</Text>
                                    </Box>
                                    <br />
                                    <Box round="xsmall" background="grey" direction='column'
                                        elevation="none" pad="xsmall" justify='center' hoverIndicator={true} onClick={() => { }}>
                                        <Text>Output 2</Text>
                                    </Box>
                                    <br />
                                    <Box round="xsmall" background="grey" direction='column'
                                        elevation="none" pad="xsmall" justify='center' hoverIndicator={true} onClick={() => { }}>
                                        <Text>Output 3</Text>
                                    </Box>
                                </Box>


                            </AccordionPanel>
                        </Accordion>
                    </div>



                    {/* <div style={{ paddingTop: '4%' }}>
            <Box background="grey" round="small" pad="xsmall">
              <div style={{ display: 'flex', flexDirection: 'row', justifyContent: "center", justifyItems: "center" }}>
                <Text style={{ paddingRight: '42%' }}>Placement :</Text>
              </div>
              <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'flex-start', paddingLeft: '30%' }}> */}
                    {/* <PositionInput value={[machine.pose_offset.position.x, machine.pose_offset.position.y, machine.pose_offset.position.z]}
          onChange={e => setItemProperty('machine', machine.uuid, 'position', { ...machine.pose_offset.position, x: e[0], y: e[1], z: e[2] })} />
        <OrientationInput value={[machine.pose_offset.orientation.w, machine.pose_offset.orientation.x, machine.pose_offset.orientation.y, machine.pose_offset.orientation.z]}
          onChange={e => setItemProperty('machine', machine.uuid, 'orientation', { ...machine.pose_offset.orientation, w: e[0], x: e[1], y: e[2], z: e[3] })} /> */}
                    {/* <PositionInput />
                <OrientationInput />
              </div>

              <br />
            </Box>
          </div> */}


                </Box>
            </Box>




        </>

    )
}
