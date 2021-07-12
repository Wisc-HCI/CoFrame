import React, { useCallback, useState } from 'react';

import useEvdStore from '../../stores/EvdStore';


import { Divider, Input, Switch, Space, Card, Button } from 'antd';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';

export const MachineInOutRegionDetail = ({ uuid }) => {
    const { region } = useEvdStore(useCallback(state => ({
        region: state.data.regions[uuid],

    })
        , [uuid]))
    const { TextArea } = Input;

    const { setItemProperty } = useEvdStore(state => ({
        setItemProperty: state.setItemProperty
    }));


    let defaultShape = "Sphere"
    if (!region.uncertainty_radius) {
        defaultShape = "Cube"
    }


    const [shape, setShape] = useState(defaultShape);
    function changeShape(newShape) {
        if (newShape !== shape) {
            if (shape === 'Cube') {
                let temp = region.uncertainty_x;
    
                setItemProperty('region', region.uuid, 'uncertainty_radius', temp)
                setItemProperty('region', region.uuid, 'uncertainty_x', null)
                setItemProperty('region', region.uuid, 'uncertainty_y', null)
                setItemProperty('region', region.uuid, 'uncertainty_z', null)
                setShape('Sphere')
            } else {
                let temp = region.uncertainty_radius;
    
                setItemProperty('region', region.uuid, 'uncertainty_x', temp)
                setItemProperty('region', region.uuid, 'uncertainty_y', temp)
                setItemProperty('region', region.uuid, 'uncertainty_z', temp)
                setItemProperty('region', region.uuid, 'uncertainty_radius', null)
                setShape('Cube')
            }
        }
    }




    let dimension = null;
    if (shape === 'Cube') {
        dimension = (
            <Card size='small'>
                <Space>
                    <div style={{ display: "flex", alignItems: 'center' }}>
                        <b style={{ color: "red", paddingRight: '5px' }}>X: </b>
                        <Input
                            disabled={!region.editable}
                            value={region.uncertainty_x}
                            style={{ width: '40%' }}

                        />
                    </div>
                    <div style={{ display: "flex", alignItems: 'center' }}>
                        <b style={{ color: "lime", paddingRight: '5px' }}>Y: </b>
                        <Input
                            disabled={!region.editable}
                            style={{ width: '40%' }}
                            value={region.uncertainty_y}

                        />
                    </div>
                    <div style={{ display: "flex", alignItems: 'center' }}>
                        <b style={{ color: "blue", paddingRight: '5px' }}>Z: </b>
                        <Input
                            disabled={!region.editable}
                            style={{ width: '40%' }}
                            value={region.uncertainty_z}

                        />
                    </div>
                </Space>
            </Card>)
    } else {
        dimension = (
            <div>
                <Card size='small'>
                    <t style={{ color: "rgba(255, 255, 255, 0.85)", paddingRight: '5px' }}>Radius: </t>
                    <Input
                        disabled={!region.editable}
                        style={{ width: '10%' }}
                        value={region.uncertainty_radius}

                    />
                </Card>
            </div>
        )

    }



    return (
        <>
            <TextArea
                defaultValue={region.description}
                disabled={!region.editable}
            />

            <Divider orientation="left" style={{ color: 'white', borderTopColor: 'rgba(255,255,255,0.12)', lineHeight: '1.5715px', paddingTop: '20px', paddingBottom: '5px' }}>
                <span>Placement</span>
            </Divider>

            <div style={{ display: 'flex', flexDirection: 'column' }}>
                <PositionInput value={[region.center_position.x, region.center_position.y, region.center_position.z]}
                    onChange={e => setItemProperty('region', region.uuid, 'center_position', { ...region.center_position, x: e[0], y: e[1], z: e[2] })} />
                <OrientationInput value={[region.center_orientation.w, region.center_orientation.x, region.center_orientation.y, region.center_orientation.z]}
                    onChange={e => setItemProperty('region', region.uuid, 'center_orientation', { ...region.center_orientation, w: e[0], x: e[1], y: e[2], z: e[3] })} />
                <br />
                <div style={{ paddingTop: '0px', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                    <b style={{ color: 'rgba(255, 255, 255, 0.85)' }}>Free Orientation:</b>
                    <Switch disabled={!region.editable} checked={region.free_orientation} style={{ left: '-30px' }} />



                </div>
                <Divider orientation="left" style={{ color: 'white', borderTopColor: 'rgba(255,255,255,0.12)', lineHeight: '1.5715px', paddingTop: '20px', paddingBottom: '5px' }}>
                    <span>Shape</span>
                    <br />
                    <br />
                </Divider>
                <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', paddingTop: '5px' }}>
                    <b style={{ color: 'rgba(255, 255, 255, 0.85)' }}>Shape:</b>
                    <div>

                        <Button style={{marginRight:2}} disabled={!region.editable && shape!=='Cube'} type={shape === 'Cube' ? 'primary' : 'default'} onClick={() => { changeShape('Cube') }}>
                            Cube
                        </Button>
                        <Button disabled={!region.editable && shape!=='Sphere'} type={shape === 'Cube' ? 'default' : 'primary'} onClick={() => { changeShape('Sphere') }}>
                            Sphere
                        </Button>
                    </div>
                </div>

                <br />




                <b style={{ color: 'rgba(255, 255, 255, 0.85)' }}>Dimension:</b>

                {dimension}

            </div>

        </>


    )
}
