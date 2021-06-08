import React from 'react';
import { Card } from 'antd';

import frameStyles from '../../frameStyles';
import useGuiStore from '../../stores/GuiStore';

import {Safety, Quality, Performance, Business} from '../Feedback';

export const ChecklistTile = (_) => {
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
        <div style={{height:'100%',paddingLeft:10,paddingRight:10,paddingBottom:10}}>
            <Card
                style={{height:'100%'}}
                bodyStyle={{padding:0,display:'flex',flexDirection:'column'}}
                title="Checklist"
                tabList={frames}
                onTabChange={setFrame}
                tabProps={{centered:true,size:'small',style:{fontSize:10,marginTop:15}}}
            >
            <div style={{height:'100%',borderWidth:5,borderColor:frameStyles.colors[frameId]}}>
            {contentList[frameId]}
            </div>
            </Card>
        </div>
    );
};