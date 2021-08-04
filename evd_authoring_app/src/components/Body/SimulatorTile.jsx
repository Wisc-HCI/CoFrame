import React from 'react';

import { Card } from 'antd';

import {Scene} from 'robot-scene';
import useGuiStore from '../../stores/GuiStore';
import { Controls } from '../Controls';
import { InfoTile } from './InfoTile';
import frameStyles from '../../frameStyles';


export const SimulatorTile = (props) => {

    const frame = useGuiStore(state => state.frame);

    return (
        <div style={{height:'100%',display:'flex',flexDirection:'column',padding:10}}>
            <Card
                style={{display:'flex',flex:2,flexDirection:'column',marginBottom:10}}
                bodyStyle={{padding:0,display:'flex',flex:1}}
                extra={<Controls/>}
                title="Simulator">
                    <div style={{flex:1,backgroundColor: frameStyles.colors[frame], padding: 5, width:'100%'}}>

                        <Scene
                            displayTfs={true}
                            displayGrid={true}
                            isPolar={false}
                            backgroundColor='#1e1e1e'
                            planeColor='#141414'
                            highlightColor={frameStyles.colors[frame]}
                            plane={-0.75}
                        />

                    </div>
            </Card>
            <InfoTile/>
        </div>
    );
};
