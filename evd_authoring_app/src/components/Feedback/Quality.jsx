import React from 'react';
import { List } from 'antd';
import { FeedbackSection } from './FeedbackSection';

export function Quality(_) {

    const sections = [
        {
            title: 'Missing Blocks',
            complete: true,
            enabled: true,
            items: [
                {label:'Block-01',complete: true},
                {label:'Block-02',complete: true},
                {label:'Block-03',complete: true}
            ]
        },
        {   
            title: 'Invalid Parameters',
            complete: false,
            enabled: true,
            items: [
                {label:'Skill-01',complete: true},
                {label:'Skill-02',complete: false},
                {label:'Skill-03',complete: false}
            ]
        },
        {
            title: 'Unused Skills',
            complete: false,
            enabled: false,
            items: []
        },
        {
            title: 'Unused Features',
            complete: false,
            enabled: false,
            items: []
        },
        {
            title: 'Empty Blocks',
            complete: false,
            enabled: false,
            items: []
        },
        {
            title: 'Reachability',
            complete: false,
            enabled: false,
            items: []
        },
    ]

    return (
        <List
        split={false}
        dataSource={sections}
        renderItem={(section)=>(
          <FeedbackSection {...section}/>
        )}
      />
    )
}