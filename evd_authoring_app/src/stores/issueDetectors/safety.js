import { generateUuid } from "../generateUuid";
export const findEndEffectorPoseIssues = (_) => { // Requires trace pose information
    let issues = {};

    return [issues, {}];
}

export const findCollisionIssues = (_) => { // Requires collision graders
    let issues = {};

    return [issues, {}];
}

export const findOccupancyIssues = (_) => { // Requires occupancy zone graders
    let issues = {};

    return [issues, {}];
}

export const findPinchPointIssues = (_) => { // Requires pinch-point graders
    let issues = {};

    return [issues, {}];
}

export const findThingMovementIssues = ({program,unrolled}) => { // May require trace pose information
    let issues = {};
    
    Object.values(unrolled).forEach(primitive=>{
        
        console.log(primitive.type);
        if (primitive.type === 'node.primitive.gripper.' && primitive.parameters.semantic === 'grasping'){
            const thingUUID = primitive.parameters.thing_uuid.uuid;
            console.log(thingUUID + "  123" );
            Object.values(program.data.placeholders).forEach(placeholder => {
                console.log(placeholder.uuid );
                if (placeholder.uuid === thingUUID){
                  const thingTypeUUID = placeholder.pending_node.thing_type_uuid;
                  console.log("true");
                  Object.values(program.data.thingTypes).forEach(thingType => {
                    console.log(thingType.uuid + "   " + thingTypeUUID);
                      if(thingType.uuid === thingTypeUUID && thingType.is_safe === false){
                        
                        const uuid = generateUuid('issue');
                        issues[uuid] = {
                        uuid: uuid,
                        requiresChanges: true,
                        title: `Grasping on unsafe object`, // might need some changes
                        description: `The robot is grasping on a dangerous item`,// might need some changes
                        complete: false,
                        focus: {uuid:primitive.uuid, type:'primitive'},
                        graphData: null
            }
                      }else{
                          console.log("false");
                      }
                })
                }
            })
            
           
            
        }
    })


    return [issues, {}];
}