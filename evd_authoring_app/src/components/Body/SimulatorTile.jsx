import React from 'react';

import { Card } from 'antd';

import {Scene} from 'robot-scene';
import useStore from '../../stores/Store';
import { Controls } from '../Controls';
import { InfoTile } from './InfoTile';


export const SimulatorTile = (_) => {

    const primaryColor = useStore(state => state.primaryColor);

    return (
        <div style={{height:'calc(100vh - 48pt)',padding:10}}>
            <Card
                style={{height:564,marginBottom:10}}
                headStyle={{height:65}}
                bodyStyle={{padding:0,height:500,margin:0,}}
                extra={<Controls/>}
                title="Simulator">
                    <div 
                        style={{height:'100%',backgroundColor: primaryColor, padding: 5, width:'100%'}}
                        
                    >
                        <Scene
                            displayTfs={false}
                            displayGrid={true}
                            isPolar={false}
                            backgroundColor='#1e1e1e'
                            planeColor='#141414'
                            highlightColor={primaryColor}
                            plane={-0.75}
                            fov={50}
                            store={useStore}
                            onPointerMissed={(e)=>{e.stopPropagation();useStore.getState().clearFocusItem();}}
                        />

                    </div>
            </Card>
            <InfoTile/>
        </div>
    );
};
