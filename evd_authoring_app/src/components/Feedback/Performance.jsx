import React from 'react';
import { List } from 'antd';
import { FeedbackSection } from './FeedbackSection';

export function Performance(_) {

    const sections = [
        {
            title: 'Joint Speed',
            complete: true,
            enabled: true,
            items: [
                {label:'Waypoint-01',complete: true},
                {label:'Waypoint-02',complete: true},
                {label:'Waypoint-03',complete: true}
            ]
        },
        {   
            title: 'End Effector Speed',
            complete: false,
            enabled: true,
            items: [
                {label:'Trajectory-01',complete: true},
                {label:'Trajectory-02',complete: false},
                {label:'Trajectory-03',complete: false}
            ]
        },
        {
            title: 'Payload',
            complete: false,
            enabled: false,
            items: []
        },
        {
            title: 'Occupancy',
            complete: false,
            enabled: false,
            items: []
        },
        {
            title: 'Pinch Points',
            complete: false,
            enabled: false,
            items: []
        },
        {
            title: 'Space Usage',
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