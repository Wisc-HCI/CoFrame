import React from 'react';
import useStore from '../../stores/Store';
import { Box } from "grommet";
import Collapse from '../Collapse';
import {Toggle} from "../Toggle";


function LocationWaypointDetail(props) {
   
    const item = useStore(state => state.programData[props.itemID]);
    const robotAgentKey = Object.keys(item.properties.reachability)[0];
   
    function displayList(){
        for (const [key, value] of Object.entries(item.properties.reachability[robotAgentKey])) {
    
           return (
            <>
                <div key={key} style={{ "paddingBottom": "3%" }}>
                    <Box round="xsmall" background="rgba(100,100,100,0.3)" direction='row'
                        elevation="none" pad="xsmall" justify='between'
                        hoverIndicator={true} >
                        <b style={{ color: 'rgba(255, 255, 255, 0.85)' }}> {key} : </b>
                        <div>
                            <Toggle selected = {value} disabled = {true}/>
                            
                       
                        </div>
                    </Box>
                </div>

            </>

        )
        }
         
    }

    return (

        <>
            
            <Collapse
                openable={true}
                borderWidth={3}
                defaultOpen={true}
                style={{ backgroundColor: '#303030', marginBottom: 5 }}
                backgroundColor='#202020'
                header={<Box direction='row' pad="10pt">
                    <b style={{ color: 'rgba(255, 255, 255, 0.85)' }} >Reachability : </b> 
                </Box>}
            >
                {displayList()}

            </Collapse>

        </>
    )

}



export default (LocationWaypointDetail);
