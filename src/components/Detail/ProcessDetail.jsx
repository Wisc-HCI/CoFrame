import React, { useCallback, useState, useEffect } from 'react';

import useStore from '../../stores/Store';

import { TextArea, Text, Box, TextInput, Button, Accordion, AccordionPanel, Layer, DropButton } from 'grommet';
import { FormClose, Trash } from 'grommet-icons';
import { DATA_TYPES, ExternalBlock, referenceTemplateFromSpec } from "simple-vp";



export const ProcessDetail = ({ item, inputOutputClick }) => {
    const data = useStore(state => state.programData);
    const clearFocusItem = useStore(state => state.clearFocusItem);
    const [inputList, setInputList] = useState([]);
    const [outputList, setOutputList] = useState([]);
    const [inputData, setInputData] = useState(null);
    const [outputData, setOutputData] = useState(null);



    useEffect(() => {
        { IterateInputOutput() }
    }, [])

    function IterateInputOutput() {
        if (item.properties.inputs.length > 0) {
            item.properties.inputs.forEach((item) => {
                setInputList(tempArr => [...tempArr, item]);
            })
        }

        if (item.properties.outputs.length > 0) {
            item.properties.outputs.forEach((item) => {
                setOutputList(tempArr => [...tempArr, item]);
            })
        }
        return null;
    }

    function RenderInputList(input) {
        if (input === true) {
            let list = [];
            let input;
            inputList.forEach((item) => {
                for (const [key, value] of Object.entries(data)) {
                    if (key === item.thingType) {
                        input = value;
                    }
                }
                //const processRef = referenceTemplateFromSpec('processType', item, objectTypeInfo);
                list.push(
                    <div key={item.id}>
                        <Box round="xsmall" background="grey" direction='column'
                            elevation="none" pad="xsmall" justify='center'
                            hoverIndicator={true} onClick={() => {
                                inputOutputClick(input.id, item.position, item.orientation);

                            }}>
                            {/* <ExternalBlock
                                store={useStore}
                                data={processRef}
                                highlightColor={"#333333"}
                            /> */}
                            <Text>{input.name}</Text>
                        </Box>
                    </div>

                );

            })
            return list;
        } else {
            let list = [];
            let output;
            outputList.forEach((item) => {
                for (const [key, value] of Object.entries(data)) {
                    if (key === item.thingType) {
                        output = value;
                    }
                }
                //const processRef = referenceTemplateFromSpec('processType', item, objectTypeInfo);
                list.push(
                    <div key={item.id}>
                        <Box round="xsmall" background="grey" direction='column'
                            elevation="none" pad="xsmall" justify='center'
                            hoverIndicator={true} onClick={() => {
                                inputOutputClick(output.id, item.position, item.orientation);
                            }}>
                            {/* <ExternalBlock
                                store={useStore}
                                data={processRef}
                                highlightColor={"#333333"}
                            /> */}
                            <Text>{output.name}</Text>
                        </Box>
                    </div>

                );

            })
            return list;
        }


    }



    return (
        <>
            <Layer full="vertical" onEsc={clearFocusItem} position="right" modal={false}>
                <Box fill style={{ minWidth: '378px' }} background='#444444'>
                    <Box
                        direction="row"
                        align="center"
                        as="header"
                        justify="between"
                        border={{ side: 'bottom', color: '#333333' }}
                    >
                        <Text margin={{ left: 'small' }} size="xlarge" style={{ textTransform: 'capitalize' }}>
                            {item.name} Information
                        </Text>
                        <Button icon={<FormClose />} onClick={clearFocusItem} />
                    </Box>
                    <Box flex overflow="auto" pad="xsmall" border={{ color: 'black', size: 'xxsmall' }}>
                        <TextInput
                            placeholder="type here"
                            value={item.name}
                            disabled={item.canEdit}
                        // onChange={e => setItemProperty(focusItem.type, focusItem.uuid, 'name', e.target.value)}
                        />
                        <br />
                        <div>
                            <Box round="xsmall" pad="small" background="#303030" >
                                <b style={{ color: 'rgba(255, 255, 255, 0.85)', paddingBottom: '2%' }} >Description : </b>
                                <div>
                                    <TextArea
                                        value={item.properties.description}
                                        disabled={!item.canEdit}
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
                                                <TextInput style={{ maxWidth: 80 }} value={item.properties.processTime} disabled={!item.canDelete} />
                                                <Text style={{ paddingLeft: "4%" }}>sec</Text>
                                            </div>
                                        </div>
                                    </Box>

                                    <div style={{ paddingTop: '4%' }}>
                                        <Accordion>
                                            <AccordionPanel label="Inputs :">
                                                <Box round="xsmall" background="grey" direction='column'
                                                    elevation="none" pad="xsmall" justify='center'>
                                                    {RenderInputList(true)}
                                                </Box>


                                            </AccordionPanel>
                                        </Accordion>
                                        <Accordion>
                                            <AccordionPanel label="Outputs :">
                                                <Box round="xsmall" background="grey" direction='column'
                                                    elevation="none" pad="xsmall" justify='center'>
                                                    {RenderInputList(false)}
                                                </Box>


                                            </AccordionPanel>
                                        </Accordion>
                                    </div>

                                </Box>
                            </Box>
                        </div>

                    </Box>
                    <Box
                        as="footer"
                        border={{ side: 'top', color: '#333333' }}
                        pad="small"
                        justify="end"
                        direction="row"
                        align="center"
                    >
                        <div style={{ marginInline: "30%", display: 'flex' }}>
                            {item.canDelete ? (
                                <div style={{ display: 'flex' }}>
                                    <DropButton secondary icon={<Trash />}
                                        dropAlign={{ bottom: 'top', right: 'right' }}
                                        dropProps={{ elevation: 'none' }}
                                        dropContent={
                                            <Box
                                                background="grey"
                                                pad="small"
                                                round="xxsmall"
                                                border={{ color: 'white', size: 'xsmall' }}
                                                align="center"
                                                elevation="none"
                                                justify="center"

                                            >
                                                <Text>
                                                    Are you sure you want to delete this item?
                                                </Text>
                                                <div style={{ paddingTop: "5%" }}>
                                                    <Button primary icon={<Trash />} label="Delete" color="#ab4646" />

                                                </div>

                                            </Box>
                                        }

                                        disabled={!item.canDelete}

                                        label="Delete" color="#ab4646"
                                    />
                                </div>
                            ) : (
                                <Button secondary icon={<Trash />} disabled={!item.canDelete} label="Delete" color="#ab4646" />
                            )}
                        </div>
                    </Box>
                </Box>
            </Layer>




        </>

    )
}
