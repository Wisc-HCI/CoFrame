import React, {useCallback, useState} from 'react';
import { Input, Row, Button, Dropdown, Menu } from 'antd';
import Icon, { EllipsisOutlined, UnlockOutlined, LockOutlined } from '@ant-design/icons';
// import { ItemSortable } from './Wrappers';
import { NodeList } from '../NodeList';
import { Base } from './Base';
import useStore from '../../../stores/Store';
import shallow from 'zustand/shallow';
import { acceptLookup } from '../acceptLookup';
import blockStyles from '../blockStyles';
import { ReactComponent as ContainerIcon } from '../../CustomIcons/Container.svg'
import '../highlight.css';
import { generateUuid } from '../../../stores/generateUuid';
// import { EditableTagGroup } from './Tags/EditableTagGroup';
import {ReactComponent as LocationIcon} from '../../CustomIcons/Location.svg';
import {ReactComponent as MachineIcon} from '../../CustomIcons/Gear.svg';
import {ReactComponent as ThingIcon} from '../../CustomIcons/Thing.svg';
import lodash from 'lodash';

const DEFAULT_ARG_NAMES = {
    "machine": 'Machine Parameter',
    "trajectory": 'Trajectory Parameter',
    "thing": 'Thing Parameter',
    "waypoint": 'Waypoint Parameter',
    "location": 'Location Parameter'
}

export const CanvasBlock = ({staticData,uuid,ancestors,context,dragDisabled,dropDisabled,dragHandle,onCanvas}) => {

    const [focusItem,setItemProperty] = useStore(state=>([state.focusItem,state.setItemProperty]),shallow);
    const isDragging = useStore(store=>store.isDragging);

    const createSkillArgument = useStore(state=>state.createSkillArgument);

    const data = useStore(useCallback((state)=>{
        return staticData ? staticData : state.data[uuid];
    },[staticData,uuid]),shallow)

    // Extend the current context with any arg-based values
    const currentContext = data.type === 'program' ? 
        {...context} 
        : 
        {...context, ...lodash.zipObject(
            data.arguments.map(argument=>argument.uuid),
            data.arguments
        )}
    
    const focused = focusItem.uuid === data.uuid;
    const [editing, setEditing] = useState(false);

    const fieldData = acceptLookup['skill'].children;

    const inDrawer = ancestors[0].uuid === 'drawer';
    const editingEnabled = !inDrawer && data.editable;

    const blockAncestors = [
        {uuid:data.uuid,...fieldData},
        ...ancestors
    ];

    // Code for creating skill arguments
    const menuClickCreateArgument = e => {
        const item = {uuid: generateUuid('skill-arg'), name: DEFAULT_ARG_NAMES[e.key], description: "", is_list: false, parameter_type: e.key, type: 'node.skill-argument.', editable: data.editable, deleteable: true};
        createSkillArgument(data.uuid, item);
    }

    return (
        <Base
          isDragging={isDragging}
          onCanvas={onCanvas}
          dragHandle={dragHandle}
          dragDisabled={dragDisabled}
          uuid={data.uuid}
          focused={focused}
          locked={data.readonly}
          name={data.name}
          nameEditable={editing}
          onNameChange={(v)=>setItemProperty(data.uuid, 'name', v)}
          type={data.type}
          extra={editingEnabled && 
            <Dropdown overlay={
                <Menu>
                    {editingEnabled && !editing && <Menu.Item key='show' onClick={() => setEditing(true)}>
                        Enable Editing
                    </Menu.Item>}
                    {editing && <Menu.Item key='show' onClick={() => setEditing(false)}>
                        Disable Editing
                    </Menu.Item>}
                    {editing && <Menu.Item key="node.machine." onClick={menuClickCreateArgument} icon={<Icon style={{marginRight:10}} component={MachineIcon}/>}>
                        New Machine Parameter
                    </Menu.Item>}
                    {editing && <Menu.Item key="node.pose.waypoint.location." onClick={menuClickCreateArgument} icon={<Icon style={{marginRight:10}} component={LocationIcon}/>}>
                        New Location Parameter
                    </Menu.Item>}
                    {editing && <Menu.Item key="node.pose.thing." onClick={menuClickCreateArgument} icon={<Icon style={{marginRight:10}} component={ThingIcon}/>}>
                        New Thing Parameter
                    </Menu.Item>}
                    {editing && <Menu.Item key="node.trajectory." onClick={menuClickCreateArgument} icon={<Icon style={{marginRight:10}} component={ContainerIcon}/>}>
                        New Tracjectory Parameter
                    </Menu.Item>}
                </Menu>
                }>
                <Button
                    type='text'
                    style={{ marginLeft: 2 }}
                    icon={<EllipsisOutlined />}
                />
            </Dropdown>}
        >
          <NodeList ancestors={blockAncestors} uuids={data.children} field='children' context={currentContext} dragDisabled={data.readonly || dragDisabled} dropDisabled={dropDisabled}/>
        </Base>
      )
};