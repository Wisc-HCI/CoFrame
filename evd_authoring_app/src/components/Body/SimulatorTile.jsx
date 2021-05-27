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
        <div style={{height:'800px',padding:10}}>
            <Card
                style={{flex:1,height:'890px'}}
                bodyStyle={{padding:0,height:'890px'}}
                title="Simulator">
                    <div style={{position:'relative',height:'830px', backgroundColor: frameStyles.colors[frame], padding: 5}}>

                        <Scene
                            displayTfs={true}
                            displayGrid={true}
                            isPolar={false}
                            backgroundColor='#1e1e1e'
                            planeColor='#141414'
                            highlightColor='#ffffff'
                            height = "700px"
                            style={{position:'relative'}}

                        />

                        <Card size="small" style ={{ position:'absolute',bottom: '-24px',left:'50%',transform: 'translate(-50%, -50%)'}}>
                        <Controls style={{ float: 'center'}}/>

                        </Card>



                    </div>





            </Card>
        </div>
    );
};
