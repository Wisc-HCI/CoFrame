import React, {useCallback, useState} from 'react';
import { Input, Row, Button, Dropdown, Menu } from 'antd';
import Icon, { EllipsisOutlined, UnlockOutlined, LockOutlined } from '@ant-design/icons';
// import { ItemSortable } from './Wrappers';
import { NodeList } from '../NodeList';
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

export const CanvasBlock = ({staticData,uuid,ancestors,context,dragDisabled,listeners,attributes}) => {

    const [frame,focusItem,deleteBlock,moveChildPrimitive,insertChildPrimitive, setItemProperty] = useStore(state=>(
        [state.frame,state.focusItem,state.deleteBlock,
        state.moveChildPrimitive,state.insertChildPrimitive,state.setItemProperty]),shallow);
    
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
    // const toggleSkillEditable = useStore(state=>state.toggleSkillEditable);

    const fieldData = acceptLookup['skill'].children;

    const inDrawer = ancestors[0].uuid === 'drawer';
    const editingEnabled = !inDrawer && data.editable;

    const blockAncestors = [
        {uuid:data.uuid,...fieldData},
        ...ancestors
    ];

    // Code for handling the draggability of the skill node itself
    // const [{ isDragging }, drag, preview] = useDrag(() => ({
    //     type: data.type,
    //     item: { ...data, parentData, dragBehavior, onDelete},
    //     options: { dragEffect: dragBehavior },
    //     collect: monitor => ({
    //       isDragging: monitor.isDragging()
    //     })
    // }))

    // Code for handling how primitives are handled when dropped in
    // const primitiveDrop = (dropData, idx) => {
    //     if (dropData.parentData.uuid === uuid && dropData.dragBehavior === 'move') {
    //         const newIdx = dropData.idx <= idx ? idx - 1 : idx;
    //         if (newIdx === dropData.idx) {
    //             return
    //         }
    //         moveChildPrimitive(dropData.uuid, dropData.parentData.uuid, uuid, dropData.idx, newIdx);
    //     } else if (dropData.dragBehavior === 'move') {
    //         moveChildPrimitive(dropData.uuid, dropData.parentData.uuid, uuid, dropData.idx, idx)
    //     } else {
    //         insertChildPrimitive(dropData, uuid, idx);
    //     }
    // }

    // Code for creating skill arguments
    const menuClickCreateArgument = e => {
        const item = {uuid: generateUuid('skill-arg'), name: DEFAULT_ARG_NAMES[e.key], description: "", is_list: false, parameter_type: e.key, type: 'node.skill-argument.', editable: data.editable, deleteable: true};
        createSkillArgument(data.uuid, item);
    }

    const onChildDelete = (dropData) => {
        deleteBlock(dropData,data.uuid)
      }

    const dragBlockStyles = {
        display:'inline-block',
        // position: ancestors[0].uuid === 'drawer' ? 'relative' : 'absolute',
        // left:data.transform.x,
        // top:data.transform.y,
        backgroundColor:
          blockStyles[data.type],
        minHeight: 30,
        minWidth: 260,
        borderRadius: 3,
        padding: 5,
        zIndex: focused ? 100 : 1,
        // opacity: isDragging ? 0.4 : 1
    };

    const handleStuff = listeners && attributes ? {
        ...listeners,
        ...attributes
    } : {}

    return (
        <div style={dragBlockStyles} className={focused?`focus-${frame}`:null}>
            <Row style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
                <Row className='custom-drag-handle' {...handleStuff} wrap={false} align='middle' style={{ boxShadow: editing ? 'inset 0px 0px 2px 1px #ffffff' : null, backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start', flex: 1, minWidth: 130, maxWidth: 200, cursor: editing ? null : "grab", zIndex: 101, marginRight: 5, height: 32 }}>
                    <Icon style={{ marginLeft: 4 }} component={ContainerIcon} />
                    <Input style={{ maxWidth: 200, color: 'white', cursor: editing ? 'text' : "grab" }} bordered={false} disabled={!editing} value={data.name} onChange={(e)=>setItemProperty('skill', data.uuid, 'name', e.target.value)} />
                </Row>
                <Row className="nodrag" wrap={false} align='middle' style={{textAlign:'end'}}>
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
            {/* <Row>
                {!inDrawer && data.type === 'skill' && <EditableTagGroup skill={data} ancestors={blockAncestors}/>}
            </Row> */}
            <NodeList ancestors={blockAncestors} uuids={data.children} context={currentContext} dragDisabled={data.readonly || dragDisabled}/>
        </div>
    )
};