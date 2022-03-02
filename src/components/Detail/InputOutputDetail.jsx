import React, { useCallback } from 'react';

import useStore from '../../stores/Store';

import { Toggle } from '../Toggle';
import { TextArea, Text, Box, TextInput, Button, Accordion, AccordionPanel, Layer, DropButton } from 'grommet';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';
import { FormClose, Trash } from 'grommet-icons';


export const InputOutputDetail = ({ item, position, orientation }) => {

    console.log("InputOutputDetial", item);
    const data = useStore(state => state.programData);
    const clearFocusItem = useStore(state => state.clearFocusItem);


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
