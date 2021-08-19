import React from 'react';
import { Alert, Tooltip } from 'antd';

export function getLocationInfo({frame,primaryColor,focusData,currentIssue}) {
    if (currentIssue) {
        return [
            <span><b style={{color:primaryColor}}>Issue: </b>{currentIssue.title}</span>, 
            <div>
                <Alert showIcon description={currentIssue.description}></Alert>
                {currentIssue.description.includes('reachable') && (
                    <span>
                        <br></br>Given the <Tooltip placement="top" title="The physical structure (joints and links) and how they impact the movement of the robot"><span style={{color:primaryColor}}>kinematics</span></Tooltip> and placement of the robot, certain locations or waypoints may not be reachable. 
                        Much like how reaching around to touch your back may be difficult (even if the location is clearly close to your body), 
                        robots can encounter difficulties accessing certain positions or orientations in the work cell. 
                        We have identified that the specific location you specified is not reachable either because it is too far from the robot base 
                        (the robot simply cannot reach that far away), or because the position is not able to be reached in that orientation. 
                        <br></br><br></br>
                    </span>
                )}
                <Alert description='Since this location is used in the program, you will need to address this issue in order to simulate the program, since it would prevent the robot from performing motions that use it.'></Alert>
            </div>
        ]
    } else {
        return [
            <span>{focusData ? focusData.name : 'Locations'}</span>,
            <div>
                {focusData && (
                    <>
                    <div style={{borderRadius:4,backgroundColor:'rgba(200,200,200,0.15)',padding:20}}>
                    {focusData.description}
                </div>
                <br></br>
                    </>
                )}
                Locations are meaningful positions in the scene. For example, they can be used to define goals for placing or picking up 
                <Tooltip placement="top" title="A part or piece that has been defined in the setup tab. These may be created, consumed, or modified by machines">
                    <span style={{color:primaryColor}}>
                        {" "}Things
                    </span>
                </Tooltip>, or specifying starting or ending positions for the robot.
                {frame === 'safety' && (
                    <Alert style={{marginTop:20}} showIcon message={<span style={{color:'white'}} >Safety Concerns</span>} description="Pay special attention to placing locations around the occupancy zone of the human, since this is more likely to result in undesirable conflicts between the human and the robot."/>
                )}
                {frame === 'performance' && (
                    <Alert style={{marginTop:20}} showIcon message={<span style={{color:'white'}} >Robot Performance</span>} description="Pay special attention to the distances between start and end locations when creating trajectories. When spaced out far between one another, you may have less control over the way that the robot moves between the two locations, resulting in suboptimal results. Consider adding waypoints between them for greater control."/>
                )}
            </div>
        ]
    }
}