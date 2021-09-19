import React, {useCallback} from 'react';
import useStore from '../../../stores/Store';
import { typeToKey } from '../../../stores/helpers';

import { EditableTag } from './EditableTag';


export const EditableTagGroup = (props) => {
  const data = useStore(useCallback(state=>state.data[typeToKey('skill')][props.skill.uuid].arguments));

  const deleteSkillArgument = useStore(state=>state.deleteSkillArgument);

  const closeTag = argument => {
    deleteSkillArgument(props.skill.uuid, argument);
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

  return (
    <div>
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