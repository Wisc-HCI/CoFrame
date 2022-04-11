import React from 'react';
// import { Tooltip } from 'antd';
import { Blurb } from './Blurb';
import { Glossary } from './Glossary';




export function getLocationInfo({frame,primaryColor,focusData,currentIssue,description}) {
  

    let tabs = [];
    if (currentIssue) {
        tabs.push(
            {
                title: currentIssue.title, 
                contents:<div>
                    <Blurb highlight={primaryColor}>
                        <h3>{currentIssue.description}</h3>
                        {currentIssue.description.includes('reachable') && (
                        <span>
                            <br></br>Given the <Glossary.Kinematics primaryColor={primaryColor}/> and placement of the robot, certain locations or waypoints may not be reachable. 
                            Much like how reaching around to touch your back may be difficult (even if the location is clearly close to your body), 
                            robots can encounter difficulties accessing certain positions or orientations in the work cell. 
                            We have identified that the specific location you specified is not reachable either because it is too far from the robot base 
                            (the robot simply cannot reach that far away), or because the position is not able to be reached in that orientation.
                        </span>
                        )}
                        {currentIssue.requiresChanges && (
                            <span>
                                <br></br>
                                <br></br>
                                <i>Since this location is used in the program, you will need to address this issue in order to simulate the program, since it would prevent the robot from performing motions that use it.</i>
                            </span>
                        )}
                    </Blurb>
                </div>
            }
        )
    }; 

    tabs.push(
        {
            title:<span>{focusData ? focusData.name : 'Locations'}</span>,
            contents:<div>
                {focusData && (
                    <Blurb highlight="rgb(50,50,50)">
                        <h3>About this Location</h3>
                        {description}
                    </Blurb>
                )}
                <Blurb highlight="rgb(50,50,50)">
                    <h3>What are Locations?</h3>
                    Locations are meaningful positions in the scene. For example, they can be used to define goals for placing or picking up {' '}
                    <Glossary.Things primaryColor={primaryColor}/>, or specifying starting or ending positions for the robot.
                </Blurb>
                
                {frame === 'safety' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Safety Concerns</h3>
                        Pay special attention to placing locations around the occupancy zone of the human, since this is more likely to result in undesirable conflicts between the human and the robot.
                    </Blurb>
                )}
                {frame === 'performance' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Robot Performance</h3>
                        Pay special attention to the distances between start and end locations when creating trajectories. When spaced out far between one another, you may have less control over the way that the robot moves between the two locations, resulting in suboptimal results. Consider adding waypoints between them for greater control.
                    </Blurb>
                )}
            </div>
        }
    )
    return tabs
}