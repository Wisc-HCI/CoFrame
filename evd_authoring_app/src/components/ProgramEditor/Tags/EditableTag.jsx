import React, { useState } from 'react';
import useStore from '../../../stores/Store';
import { UUIDBlock } from '../UUIDBlock';

export const EditableTag = (props) => {

    const data = props.data;
    const context = [{name:data.uuid, real:true}];
    const setArgumentProperty = useStore(state=>state.setArgumentProperty);
  
    const handleEditInputConfirm = newName => {
      setArgumentProperty(props.parent, props.id, 'name', newName);
    };

    const handleDeleteClick = e => {
      props.closefunc(props.data);
      e.preventDefault();
    }

    const itemType = props.itemType;

    return (
      <UUIDBlock 
          ancestors={props.ancestors}
          parentData={{type:'skill',uuid:props.parent}}
          data={{...data, itemType, type:props.type}} 
          context={context}
          editable={props.editable}
          onNameChange={handleEditInputConfirm}
          onDelete={handleDeleteClick}>
      </UUIDBlock>
    );
}