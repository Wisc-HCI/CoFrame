import React from 'react';
import useStore from '../../stores/Store';
import { Box, Button } from "grommet";
import Collapse from '../Collapse';
import {Toggle} from "../Toggle";
import { FiRefreshCw } from 'react-icons/fi';


function LocationWaypointDetail(props) {
   
    const item = useStore(state => state.programData[props.itemID]);
    const robotAgents = useStore(state => Object.values(state.programData).filter(item=>item.type === 'robotAgentType'));
    const grippers = useStore(state => Object.values(state.programData).filter(item=>item.type === 'gripperType'));
    const primaryColor = useStore(state=>state.primaryColor);
    const forceRefreshBlock = useStore(state=>state.forceRefreshBlock);
   
    return (
        <Collapse
            openable={true}
            borderWidth={10}
            defaultOpen={true}
            style={{ backgroundColor: '#303030', marginBottom: 5 }}
            backgroundColor='#202020'
            header={
                <Box direction='row'>
                    <b style={{ color: 'rgba(255, 255, 255, 0.85)' }} >Reachability : </b> 
                </Box>
            }
            extra={
                <Button secondary onClick={()=>forceRefreshBlock(item.id)} icon={<FiRefreshCw color={primaryColor}/>} margin={{right:'small'}}/>
            }
        >
            {Object.keys(item.properties.reachability).length===0 && (
                <i>Reachability Loading...</i>
            )}
            {robotAgents.map(robotAgent=>{
                if (item.properties.reachability[robotAgent.id]) {
                    return (
                        <Box key={robotAgent.id} background='black' round='xsmall' gap='xsmall' pad='xsmall'>
                            {robotAgent.name}
                            {grippers.map(gripper=>{
                                if (item.properties.reachability[robotAgent.id][gripper.id] !== undefined) {
                                    return (
                                        <Box direction='row' justify='between' align='center' key={gripper.id} background='#333333' round='xsmall' gap='xsmall' pad='xsmall'>
                                            {gripper.name}
                                            <Toggle selected={item.properties.reachability[robotAgent.id][gripper.id]} disabled={true} backgroundColor={primaryColor} size='small'/>
                                        </Box>
                                    )
                                    
                                    
                                }
                            })}
                        </Box>
                    )
                }
            })}

        </Collapse>
    )

}



export default (LocationWaypointDetail);
