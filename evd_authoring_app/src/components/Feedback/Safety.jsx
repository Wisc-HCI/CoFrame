import React from 'react';
import { List } from 'antd';
import { FeedbackSection } from './FeedbackSection';

export function Safety(_) {

    const sections = [
        {
            title: 'Pinch Points',
            complete: true,
            enabled: true,
            items: [
                {label:'Waypoint-01',complete: true},
                {label:'Waypoint-02',complete: true},
                {label:'Waypoint-03',complete: true}
            ]
        },
        {   
            title: 'Collisions',
            complete: false,
            enabled: true,
            items: [
                {label:'Trajectory-01',complete: true},
                {label:'Trajectory-02',complete: false},
                {label:'Trajectory-03',complete: false}
            ]
        },
        {
            title: 'Occupancy',
            complete: false,
            enabled: false,
            items: []
        },
        {
            title: 'Thing Movement',
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