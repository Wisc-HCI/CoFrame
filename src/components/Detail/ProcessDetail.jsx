import React from 'react';
import useStore from '../../stores/Store';
import {Text, Box} from 'grommet';
import {ExternalBlock, referenceTemplateFromSpec } from "simple-vp";
import Collapse from '../Collapse';
import { TextInput } from 'grommet';

export const ProcessDetail = ({process}) => {
    const inputOutputTypeInfo = useStore(state => state.programSpec.objectTypes.inputOutputType);
    
    const addFocusItem = useStore(state => state.addFocusItem);


    const programData = useStore(state => state.programData);

    function RenderInputList(input) {
        if (input === true) {
            let list = [];
            process.properties.inputs.forEach((ioItem) => {
                const thingTypeItem = Object.values(programData).filter(value => value.id === ioItem.thingType);
                const processRef = referenceTemplateFromSpec('inputOutputType', thingTypeItem[0], inputOutputTypeInfo);
                list.push(
                    <div key={ioItem.thingType}>
                        <Box round="xsmall" background="rgba(100,100,100,0.3)" direction='column'
                            elevation="none" pad="xsmall" justify='center'
                            hoverIndicator={true} onClick={() => {
                                addFocusItem(ioItem.thingType,true);

                            }}>
                            <ExternalBlock
                                store={useStore}
                                data={processRef}
                                highlightColor={"#333333"}
                            />
                        </Box>
                    </div>

                );

            })
            if (list.length > 0) {
                return list;
            } else {
                return null;
            }

        } else {
            let list = [];
            
            process.properties.outputs.forEach((ioItem) => {      
                const thingTypeItem = Object.values(programData).filter(value => value.id === ioItem.thingType);
                const processRef = referenceTemplateFromSpec('inputOutputType', thingTypeItem[0], inputOutputTypeInfo);
                list.push(
                    <div key={ioItem.thingType}>
                        <Box round="xsmall" background="rgba(100,100,100,0.3)" direction='column'
                            elevation="none" pad="xsmall" justify='center'
                            hoverIndicator={true} onClick={() => {
                                addFocusItem(ioItem.thingType,true);
                            }}>
                            <ExternalBlock
                                store={useStore}
                                data={processRef}
                                highlightColor={"#333333"}
                            />

                        </Box>
                    </div>

                );

            })
            if (list.length > 0) {
                return list;
            } else {
                return null;
            }
        }


    }



    return (
        <>
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
                                <TextInput style={{ maxWidth: 80 }} value={process.properties.processTime} disabled={!process.canDelete} />
                                <Text style={{ paddingLeft: "4%" }}>sec</Text>
                            </div>
                        </div>
                    </Box>

                    <div style={{ paddingTop: '4%' }}>
                        <Collapse
                            openable={true}
                            borderWidth={3}
                            header={<Box direction='row' pad="10pt">Inputs : </Box>}
                            style={{ marginBottom: 5 }}
                        >
                            {RenderInputList(true)}
                        </Collapse>
                        <Collapse
                            openable={true}
                            borderWidth={3}
                            header={<Box direction='row' pad="10pt">Outputs : </Box>}
                            style={{ marginBottom: 5 }}
                        >
                            {RenderInputList(false)}
                        </Collapse>
                    </div>

                </Box>
            </Box>

        </>

    )
}
