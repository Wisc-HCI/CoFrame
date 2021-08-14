import React from 'react';
// import { Alert } from 'antd';

export function getLocationInfo({editorPane,setupTab,frame,primaryColor,focusData,secondaryFocusData,currentIssue}) {
    if (currentIssue) {
        return [
            <span><b style={{color:primaryColor}}>Issue: </b>{currentIssue.title}</span>, 
            <div>
                {currentIssue.description}
            </div>
        ]
    }
    return ['', '']
}