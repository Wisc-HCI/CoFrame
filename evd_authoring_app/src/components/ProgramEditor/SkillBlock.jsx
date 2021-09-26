import React, {useCallback, useState} from 'react';
import { Input, Row, Button, Dropdown, Menu } from 'antd';
import Icon, { EllipsisOutlined, UnlockOutlined, LockOutlined } from '@ant-design/icons';
import { SortableSeparator } from './SortableSeparator';
// import { ItemSortable } from './Wrappers';
import { NodeZone } from './NodeZone';
import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';
import { acceptLookup } from './acceptLookup';
import blockStyles from './blockStyles';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg'
import './highlight.css';
import { ActionBlock } from './ActionBlock';
import { generateUuid } from '../../stores/generateUuid';
import { EditableTagGroup } from './Tags/EditableTagGroup';
import { useDrag } from 'react-dnd';
import {ReactComponent as LocationIcon} from '../CustomIcons/Location.svg';
import {ReactComponent as MachineIcon} from '../CustomIcons/Gear.svg';
import {ReactComponent as ThingIcon} from '../CustomIcons/Thing.svg';

const DEFAULT_ARG_NAMES = {
    "node.machine.": 'Machine Parameter',
    "node.trajectory.": 'Trajectory Parameter',
    "node.pose.thing.": 'Thing Parameter',
    "node.pose.waypoint.": 'Waypoint Parameter',
    "node.pose.waypoint.location.": 'Location Parameter'
}

export const SkillBlock = ({staticData,uuid,parentData,dragBehavior,ancestors,context,onDelete}) => {

    const [frame,focusItem,deleteHierarchical,deleteChildPrimitive,
        moveChildPrimitive,insertChildPrimitive, setItemProperty] = useStore(state=>(
        [state.frame,state.focusItem,state.deleteHierarchical,state.deleteChildPrimitive,
        state.moveChildPrimitive,state.insertChildPrimitive,state.setItemProperty]),shallow);
    
    const createSkillArgument = useStore(state=>state.createSkillArgument);

    const data = useStore(useCallback((state)=>{
        return staticData ? staticData : state.data.skills[uuid];
    },[staticData,uuid]),shallow)
    
    const focused = focusItem.uuid === data.uuid;
    const [editing, setEditing] = useState(false);
    // const toggleSkillEditable = useStore(state=>state.toggleSkillEditable);

    const fieldData = acceptLookup['node.primitive.hierarchical.skill.'].primitiveIds;

    const inDrawer = ancestors[0].uuid === 'drawer';
    const editingEnabled = !inDrawer && data.editable;

    const skillAncestors = [
        {uuid:data.uuid,...fieldData},
        ...ancestors
    ];

    // Code for handling the draggability of the skill node itself
    const [{ isDragging }, drag, preview] = useDrag(() => ({
        type: data.type,
        item: { ...data, parentData, dragBehavior, onDelete},
        options: { dragEffect: dragBehavior },
        collect: monitor => ({
          isDragging: monitor.isDragging()
        })
    }))

    // Code for handling how primitives are handled when dropped in
    const primitiveDrop = (dropData, idx) => {
        if (dropData.parentData.uuid === uuid && dropData.dragBehavior === 'move') {
            const newIdx = dropData.idx <= idx ? idx - 1 : idx;
            if (newIdx === dropData.idx) {
                return
            }
            moveChildPrimitive(dropData.uuid, dropData.parentData.uuid, uuid, dropData.idx, newIdx);
        } else if (dropData.dragBehavior === 'move') {
            moveChildPrimitive(dropData.uuid, dropData.parentData.uuid, uuid, dropData.idx, idx)
        } else {
            insertChildPrimitive(dropData, uuid, idx);
        }
    }

    // Code for creating skill arguments
    const menuClickCreateArgument = e => {
        const item = {uuid: generateUuid('skill-arg'), name: DEFAULT_ARG_NAMES[e.key], description: "", is_list: false, parameter_type: e.key, type: 'node.skill-argument.', editable: data.editable, deleteable: true};
        createSkillArgument(data.uuid, item);
    }

    // Extend the current context with any arg-based values
    let currentContext = {
        ...context
    };
    data.arguments.forEach(arg=>{
        currentContext[arg.uuid] = {name:arg.name,real:false}
    })

    const onChildDelete = (dropData) => {
        if (dropData.type.includes('hierarchical')) {
          deleteHierarchical(dropData,data.uuid)
        } else {
          deleteChildPrimitive(data.uuid,dropData.uuid)
        }
      }

    const dragBlockStyles = {
        display:'inline-block',
        position: ancestors[0].uuid === 'drawer' ? 'relative' : 'absolute',
        left:data.transform.x,
        top:data.transform.y,
        backgroundColor:
          blockStyles['node.primitive.hierarchical.skill.'],
        minHeight: 30,
        minWidth: 260,
        borderRadius: 3,
        padding: 5,
        zIndex: focused ? 100 : 1,
        opacity: isDragging ? 0.4 : 1
    };

    return (
        <div ref={preview} style={dragBlockStyles} className={focused?`focus-${frame}`:null}>
            <Row style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
                <Row ref={editing ? null : drag} wrap={false} align='middle' style={{ boxShadow: editing ? 'inset 0px 0px 2px 1px #ffffff' : null, backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start', flex: 1, minWidth: 130, maxWidth: 200, cursor: editing ? null : "grab", zIndex: 101, marginRight: 5, height: 32 }}>
                    <Icon style={{ marginLeft: 4 }} component={ContainerIcon} />
                    <Input style={{ maxWidth: 200, color: 'white', cursor: editing ? 'text' : "grab" }} bordered={false} disabled={!editing} value={data.name} onChange={(e)=>setItemProperty('skill', data.uuid, 'name', e.target.value)} />
                </Row>
                <Row wrap={false} align='middle' style={{textAlign:'end'}}>
                    {editingEnabled ? <UnlockOutlined style={{marginRight:5,marginLeft:5}}/> : <LockOutlined style={{marginRight:5,marginLeft:5}}/>}
                    {editingEnabled && 
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
                </Row>
            </Row>
            <Row>
                {!inDrawer && <EditableTagGroup skill={data} ancestors={skillAncestors}/>}
            </Row>
            <NodeZone
                ancestors={skillAncestors}
                onDrop={(dropData) => primitiveDrop(dropData, 0)}
                emptyMessage='No Actions'
                dropDisabled={!editingEnabled}
                context={currentContext}
            >
                {data.primitiveIds.map((id, idx) => (
                    <React.Fragment key={idx}>
                        {idx === 0 && (
                            <SortableSeparator
                                key={0}
                                spacing={idx===0 ? 0 : 5}
                                height={30}
                                ancestors={skillAncestors}
                                context={currentContext}
                                onDrop={(dropData) => primitiveDrop(dropData, 0)}
                                dropDisabled={!editingEnabled}
                            />
                        )}
                        <ActionBlock
                            key={id}
                            uuid={id}
                            parentData={{ type: 'program', uuid, field: 'primitive_uuids' }}
                            dragBehavior='move'
                            onDelete={onChildDelete}
                            ancestors={skillAncestors}
                            context={currentContext}
                            idx={idx}
                            dragDisabled={!editingEnabled}
                            after={
                                <SortableSeparator
                                    ancestors={skillAncestors}
                                    height={30}
                                    end={idx === data.primitiveIds.length-1}
                                    spacing={idx === data.primitiveIds.length-1 ? 0 : 5}
                                    context={currentContext}
                                    onDrop={(dropData) => primitiveDrop(dropData, idx + 1)}
                                    dropDisabled={!editingEnabled}
                                />
                            }
                        />
                    </React.Fragment>
                ))}
            </NodeZone>
        </div>
    )
};