import React from 'react';
import { List } from 'antd';
import { ReviewSection } from './ReviewSection';

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
                {label:'Trajectory-01',complete: true, force:true},
                {label:'Trajectory-02',complete: false, force:true},
                {label:'Trajectory-03',complete: false, force:false}
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
          <ReviewSection {...section}/>
        )}
      />
    )
}