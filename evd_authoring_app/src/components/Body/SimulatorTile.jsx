import React from 'react';

import { Card } from 'antd';
import { Separator } from '@fluentui/react/lib/Separator';

import { Tile } from './Tile';
import { TileHeader } from './TileHeader';
// import { Simulator } from '../Simulator';
import {Scene} from 'robot-scene';
import useGuiStore from '../../stores/GuiStore';
import { Controls } from '../Controls';
import frameStyles from '../../frameStyles';


export const SimulatorTile = (props) => {

    const frame = useGuiStore(state => state.frame);
    
    return (
        <div style={{height:'100%',padding:10}}>
            <Card 
                style={{height:'100%'}}
                bodyStyle={{padding:0,display:'flex',flexDirection:'column',height:'100%'}}
                title="Simulator">
                    <div style={{flex:4, backgroundColor: frameStyles.colors[frame], padding: 5}}>
                        <Scene 
                            displayTfs={true}
                            displayGrid={true}
                            isPolar={false}
                            backgroundColor='#1e1e1e'
                            planeColor='#141414'
                            highlightColor='#ffffff'
                        />
                    </div>
                    
                    
                    <div style={{flex:1, width:'100%',padding:15}}>CONTROLS HERE</div>
            </Card>
        </div>
    );
};