import React from 'react';
import { List } from 'antd';
import { ReviewSection } from './ReviewSection';

export function Business(_) {

    const sections = [
        {
            title: 'Cycle Time',
            complete: true,
            enabled: true,
            items: [
                {label:'Waypoint-01',complete: true},
                {label:'Waypoint-02',complete: true},
                {label:'Waypoint-03',complete: true}
            ]
        },
        {   
            title: 'Idle Time',
            complete: false,
            enabled: true,
            items: [
                {label:'Trajectory-01',complete: true},
                {label:'Trajectory-02',complete: false},
                {label:'Trajectory-03',complete: false}
            ]
        },
        {
            title: 'Return on Investment',
            complete: false,
            enabled: false,
            items: []
        }
    ]

    return (
        <List
        split={false}
        dataSource={sections}
        renderItem={(section)=>(
          <ReviewSection {...section}/>
        )}
      />
    )
}