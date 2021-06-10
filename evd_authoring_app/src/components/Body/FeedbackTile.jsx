import React from 'react';
import { Card, Space, Button } from 'antd';
import { FrameButton } from '../FrameButton';

import frameStyles from '../../frameStyles';
import useGuiStore from '../../stores/GuiStore';

import {Safety, Quality, Performance, Business} from '../Feedback';

export const FeedbackTile = (_) => {
    const {frameId, setFrame} = useGuiStore(state=>({frameId:state.frame,setFrame:state.setFrame}));
    const frames = [
        {
            key:'safety',
            tab:<span style={{textAlign:'center',opacity:frameId === 'safety' ? 1 : 0.5,fontSize:12,color:frameStyles.colors['safety']}}>Safety<br/>Concerns</span>
        },
        {
            key:'quality',
            tab:<span style={{textAlign:'center',opacity:frameId === 'quality' ? 1 : 0.5,fontSize:12,color:frameStyles.colors['quality']}}>Program<br/>Quality</span>
        },
        {
            key:'performance',
            tab:<span style={{textAlign:'center',opacity:frameId === 'performance' ? 1 : 0.5,fontSize:12,color:frameStyles.colors['performance']}}>Robot<br/>Performance</span>
        },
        {
            key:'business',
            tab:<span style={{textAlign:'center',opacity:frameId === 'business' ? 1 : 0.5,fontSize:12,color:frameStyles.colors['business']}}>Business<br/>Objectives</span>
        }
    ]
    const contentList = {
      safety: <Safety/>,
      quality: <Quality/>,
      performance: <Performance/>,
      business:  <Business/>
    };
    
    return (
        <div style={{height:'100%',paddingLeft:10,paddingRight:10,paddingBottom:10,display:'flex',flexDirection:'column'}}>
            <Space style={{flexDirection:'row',display:'flex',justifyContent:'space-between',margin:5}}>
                <FrameButton onClick={()=>setFrame('safety')} frame={'safety'} active={frameId=='safety'} text='Safety Concerns'/>
                <FrameButton onClick={()=>setFrame('quality')} frame={'quality'} active={frameId=='quality'} text='Program Quality'/>
                <FrameButton onClick={()=>setFrame('performance')} frame={'performance'} active={frameId=='performance'} text='Robot Performance'/>
                <FrameButton onClick={()=>setFrame('business')} frame={'business'} active={frameId=='business'} text='Business Objectives'/>
            </Space>
            <Card
                style={{flex:1}}
                bodyStyle={{padding:0,display:'flex',flexDirection:'column'}}
                title="Review"
            >
            <div style={{height:'100%',borderWidth:5,borderColor:frameStyles.colors[frameId]}}>
            {contentList[frameId]}
            </div>
            </Card>
        </div>
    );
};