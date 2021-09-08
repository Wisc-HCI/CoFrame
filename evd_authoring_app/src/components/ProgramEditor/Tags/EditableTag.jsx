import React, {useCallback, useRef} from 'react';
import { Tag, Input} from 'antd';
import { useDrag } from 'react-dnd';
import useEvdStore, {typeToKey} from '../../../stores/EvdStore';

import Icon from '@ant-design/icons';
import {ReactComponent as LocationIcon} from '../../CustomIcons/Location.svg';
import {ReactComponent as MachineIcon} from '../../CustomIcons/Gear.svg';
import {ReactComponent as ContainerIcon} from '../../CustomIcons/Container.svg';
import {ReactComponent as ThingIcon} from '../../CustomIcons/Thing.svg';
import { EditOutlined } from '@ant-design/icons';

export const EditableTag = (props) => {
    const ref = useRef(null);

    let saveEditInputRef;
    let editInputValue;

    const data = useEvdStore(useCallback(state=>state.data[typeToKey('skill')][props.parent].parameters[props.id],[props.id,'parameter']));
    // const data = useEvdStore(useCallback(state=>state.data[typeToKey('parameter')][props.id],[props.id,'parameter']));
    const setParameterProperty = useEvdStore(state=>state.setParameterProperty);
  
    const [{opacity}, drag, preview] = useDrag(()=>({
      type: props.type,
      item: {itemType: props.itemType, uuid: props.id, type: props.type},
      collect: monitor => ({
        opacity: monitor.isDragging() ? 0.4 : 1
      })
    }))
  
    drag(ref);

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

    const tagIcon = () => {
      if (data['type'] === 'uuid-machine') {
        return <Icon component={MachineIcon}/>
      } else if (data['type'] === 'node.pose.waypoint.location.') {
        return <Icon component={LocationIcon}/>
      } else if (data['type'] === 'node.primitive.move-trajectory.') {
        return <Icon component={ContainerIcon}/>
      } else if (data['type'] === 'thing') {
        return <Icon component={ThingIcon}/>
      }
    }

    const isLongTag = data['name'] > 20;

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
        {!data['editting'] && <Tag
              icon={tagIcon()}
              ref={ref}
              style={{opacity}}
              className="edit-tag"
              key={props.id}
              closable={data["closable"]}
              onClose={() => props.closefunc(props.id)}
            >
              {isLongTag ? `${data['name'].slice(0, 20)}...` : data['name']} <Icon onClick={handleEditClick} component={EditOutlined}/>
            </Tag>}
      </div>
    );
}