import React from 'react';
import { Alert, Tooltip } from 'antd';

export function getWaypointInfo({frame,primaryColor,focusData,currentIssue}) {
    if (currentIssue) {
        return [
            <span><b style={{color:primaryColor}}>Issue: </b>{currentIssue.title}</span>, 
            <div>
                <Alert showIcon description={currentIssue.description}></Alert>
                {currentIssue.description.includes('reachable') && (
                    <span>
                        <br></br>Given the <Tooltip placement="top" title="The physical structure (joints and links) and how they impact the movement of the robot"><span style={{color:primaryColor}}>kinematics</span></Tooltip> and placement of the robot, certain locations or waypoints may not be reachable. 
                        Much like how reaching around to touch your back may be difficult (even if the waypoint is clearly close to your body), 
                        robots can encounter difficulties accessing certain positions or orientations in the work cell. 
                        We have identified that the specific waypoint you specified is not reachable either because it is too far from the robot base 
                        (the robot simply cannot reach that far away), or because the position is not able to be reached in that orientation. 
                        <br></br><br></br>
                    </span>
                )}
                <Alert description='Since this waypoint is used in the program, you will need to address this issue in order to simulate the program, since it would prevent the robot from performing motions that use it.'></Alert>
            </div>
        ]
    } else {
        return [
            <span>{focusData ? focusData.name : 'Waypoints'}</span>,
            <div>
                {focusData && (
                    <>
                    <div style={{borderRadius:4,backgroundColor:'rgba(200,200,200,0.15)',padding:20}}>
                    {focusData.description}
                </div>
                <br></br>
                    </>
                )}
                Waypoints are positions and orientations that are used as parts of
                <Tooltip placement="top" title="A specification of the robot's motion, utilizing a starting and ending Location, as well as a sequence of transitional waypoints between them. These are specified within the Program Editor as a block.">
                    <span style={{color:primaryColor}}>
                        {" "}Trajectories
                    </span>
                </Tooltip>, and unlike Locations, do not have inherent meaning other than to allow greater specificity of the manner with which the robot moves between a pair of locations.
                {frame === 'safety' && (
                    <Alert style={{marginTop:20}} showIcon message={<span style={{color:'white'}} >Safety Concerns</span>} description="Pay special attention to placing waypoints around the occupancy zone of the human, since this is more likely to result in undesirable conflicts between the human and the robot."/>
                )}
                {frame === 'performance' && (
                    <Alert style={{marginTop:20}} showIcon message={<span style={{color:'white'}} >Robot Performance</span>} description="Pay special attention to the sequences of waypoints and where they are relative to one another within a trajectory. Longer trajectories take longer to execute and can contribute to greater space usage."/>
                )}
            </div>
        ]
    }
}