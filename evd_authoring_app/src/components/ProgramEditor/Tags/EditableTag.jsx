import React from 'react';
import useStore from '../../../stores/Store';
import { UUIDBlock } from '../UUIDBlock';

export const EditableTag = (props) => {

    const data = props.data;
    const setArgumentProperty = useStore(state=>state.setArgumentProperty);
  
    const handleEditInputConfirm = newName => {
      setArgumentProperty(props.parent, props.id, 'name', newName);
    };

    const handleDeleteClick = e => {
      props.closefunc(props.data);
      if (e !== undefined) {
        e.preventDefault();
      }
    }

    const itemType = props.itemType;
    const uuid = data.uuid;

    return (
      <UUIDBlock 
          ancestors={props.ancestors}
          parentData={{type:'skill',uuid:props.parent}}
          data={{...data, itemType, type:props.type}} 
          context={{uuid: {name: uuid, real: false}}}
          editable={props.editable}
          onNameChange={handleEditInputConfirm}
          onDelete={handleDeleteClick}>
      </UUIDBlock>
    );
}