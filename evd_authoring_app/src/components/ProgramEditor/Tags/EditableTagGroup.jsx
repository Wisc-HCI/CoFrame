import React, {useCallback} from 'react';
import useStore from '../../../stores/Store';
import { typeToKey } from '../../../stores/helpers';

import { EditableTag } from './EditableTag';


export const EditableTagGroup = (props) => {
  const uuid = props.skill.uuid;
  const data = useStore(useCallback(state=>state.data[typeToKey('skill')][uuid].arguments,[uuid]));

  const deleteSkillArgument = useStore(state=>state.deleteSkillArgument);

  const closeTag = argument => {
    deleteSkillArgument(props.skill, argument);
  };

  const determineItemType = param_type => {
    if (param_type === "node.machine.") {
      return 'machine';
    } else if (param_type === "node.pose.thing.") {
      return 'thing';
    } else if (param_type === "node.trajectory.") {
      return 'trajectory';
    } else if (param_type === "node.pose.waypoint.location.") {
      return 'location';
    }
    return '';
  }

  const fieldStyle = {
    borderRadius: 4,
    width: '100%',
    marginBottom: 4,
    padding: 5,
    backgroundColor: "rgba(0,0,0,0.2)"
  }

  return (
    <div style={fieldStyle} >
      {data.map((obj) => {
          return (
            <div key={obj.uuid} style={{paddingTop:5}} >
              <EditableTag 
                key={obj.uuid} 
                id={obj.uuid} 
                data={obj}
                itemType={determineItemType(obj.parameter_type)}
                type={'uuid-'+determineItemType(obj.parameter_type)} 
                parent={props.skill.uuid} 
                editable={props.skill.editable}
                ancestors={props.ancestors} 
                closefunc={closeTag}/>
            </div>
          )
      })}
    </div>
  );
}