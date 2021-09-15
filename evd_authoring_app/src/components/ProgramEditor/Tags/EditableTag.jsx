import React, {useCallback, useRef} from 'react';
import { Input} from 'antd';
import useStore from '../../../stores/Store';
import { typeToKey } from '../../../stores/helpers';
import { UUIDBlock } from '../UUIDBlock';

export const EditableTag = (props) => {
    const ref = useRef(null);

    let saveEditInputRef;
    let editInputValue;

    const data = useStore(useCallback(state=>state.data[typeToKey('skill')][props.parent].parameters[props.id],[props.id,'parameter']));
    const context = [{name:data.uuid, real:true}];
    const setParameterProperty = useStore(state=>state.setParameterProperty);

    const handleEditInputChange = e => {
      editInputValue = e.target.value;
    };
  
    const handleEditInputConfirm = () => {
      setParameterProperty(props.parent, props.id, 'name', editInputValue);
      setParameterProperty(props.parent, props.id, 'editting', false);
      editInputValue = '';
    };

    const handleEditClick = e => {
      setParameterProperty(props.parent, props.id, 'editting', true);
      e.preventDefault();
    };

    const handleDeleteClick = e => {
      props.closefunc(props.id);
      e.preventDefault();
    }

    const itemType = data.itemType;

    return (
      <div>
        {data['editting'] && <Input
              ref={saveEditInputRef}
              key={props.id}
              size="small"
              className="tag-input"
              value={editInputValue}
              onChange={handleEditInputChange}
              onBlur={handleEditInputConfirm}
              onPressEnter={handleEditInputConfirm}
            />
        }
        {!data['editting'] && 
          <UUIDBlock 
              ancestors={props.ancestors}
              parentData={{type:'skill',uuid:props.parent}}
              data={{...data, itemType, type:data.type}} 
              context={context}
              onDelete={handleDeleteClick}>
          </UUIDBlock>}
        {/* {!data['editting'] && 
        <Tag
              icon={tagIcon()}
              ref={ref}
              style={{opacity}}
              className="edit-tag"
              key={props.id}
              closable={data["closable"]}
              onClose={() => props.closefunc(props.id)}
            >
              {isLongTag ? `${data['name'].slice(0, 20)}...` : data['name']} <Icon onClick={handleEditClick} component={EditOutlined}/>
            </Tag>} */}
      </div>
    );
}