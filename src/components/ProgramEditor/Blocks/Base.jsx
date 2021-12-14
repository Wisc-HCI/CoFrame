import React from 'react';
import { Input, Row } from 'antd';
import Icon, { UnlockOutlined, LockOutlined } from '@ant-design/icons';
import blockStyles from '../blockStyles';
import '../highlight.css';
import { ReactComponent as LocationIcon } from '../../CustomIcons/Location.svg';
import { ReactComponent as MachineIcon } from '../../CustomIcons/Gear.svg';
import { ReactComponent as ThingIcon } from '../../CustomIcons/Thing.svg';
import { ReactComponent as WaypointIcon } from '../../CustomIcons/Waypoint.svg';
import { ReactComponent as ContainerIcon } from '../../CustomIcons/Container.svg';
import { ReactComponent as PrimitiveIcon } from '../../CustomIcons/Primitive.svg';
import useStore from '../../../stores/Store';

const IconLookup = {
    'location':LocationIcon,
    'machine':MachineIcon,
    'thing':ThingIcon,
    'waypoint':WaypointIcon,
    'trajectory':ContainerIcon,
    'program':ContainerIcon,
    'skill':ContainerIcon,
    'hierarchical':ContainerIcon,
    'skill-call':PrimitiveIcon,
    'delay':PrimitiveIcon,
    'gripper':PrimitiveIcon,
    'machine-initialize':PrimitiveIcon,
    'process-start':PrimitiveIcon,
    'process-stop':PrimitiveIcon,
    'process-wait':PrimitiveIcon,
    'move-trajectory':PrimitiveIcon,
    'move-unplanned':PrimitiveIcon,
    'breakpoint':PrimitiveIcon,
}


export const Base = ({ dragHandle, dragDisabled, uuid, name, nameEditable, onNameChange, extra, focused, children, type, style, locked, onCanvas }) => {

    const frame = useStore(store=>store.frame)

    const blockStyle = {
        backgroundColor:
          blockStyles[type],
        minHeight: 25,
        minWidth: 200,
        borderRadius: 3,
        fontSize: 14,
        padding: 5,
        position: 'relative',
        margin: 0,
        textAlign: 'left',
        zIndex: focused ? 100 : 1,
        ...style
      };

    console.log(!onCanvas)

    return (
        <div style={blockStyle} className={focused ? `focus-${frame}` : null} onDrag={!onCanvas ? e=>{console.log('drag event');e.stopPropagation()} : null}>
            <Row className={dragHandle && 'nodrag'} wrap={false} style={{ fontSize: 16, marginBottom: children ? 7 : 0 }} align='middle' justify='space-between'>
                <Row className={uuid} ref={dragHandle} wrap={false} align='middle' style={{ boxShadow: nameEditable ? 'inset 0px 0px 2px 1px #ffffff' : null, backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start', flex: 1, minWidth: 130, maxWidth: 200, cursor: dragDisabled ? "not-allowed" : "grab", zIndex: 101, marginRight: 5, height: 32 }}>
                    <Icon className={dragHandle ? 'nodrag' : uuid} style={{ marginLeft: 4 }} component={IconLookup[type]} />
                    <Input className={['nodrag',nameEditable ? null : 'unselectable']} style={{maxWidth: 200, color: 'white', cursor: nameEditable ? 'text' : dragDisabled ? "not-allowed" : "grab" }} bordered={false} disabled={!nameEditable} value={name} onChange={(e) => onNameChange(e.target.value)} />
                </Row>
                <Row className="nodrag" wrap={false} align='middle' style={{ textAlign: 'end' }}>
                    {extra}
                    {locked ? <LockOutlined style={{ marginRight: 5 }}/> : <UnlockOutlined style={{ marginRight: 5 }} /> }
                </Row>
            </Row>
            {children}
        </div>
    )
}