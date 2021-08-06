import React from 'react';
import { Row, Button } from 'antd';
import Icon, { EllipsisOutlined, UnlockOutlined } from '@ant-design/icons';
import { useDrag } from 'react-dnd';
import { ItemSortable } from './Wrappers';
import { NodeZone } from './NodeZone';
import useGuiStore from '../../stores/GuiStore';
import useEvdStore from '../../stores/EvdStore';
import blockStyles from './blockStyles';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg'
import './highlight.css';

import { acceptLookup } from './acceptLookup';

export const ProgramBlock = (props) => {

    const [uuid, name, type, transform, primitiveIds] = useEvdStore(state=>([
        state.uuid,
        state.name,
        state.type,
        state.transform,
        state.primitiveIds
    ]))

    const moveChildPrimitive = useEvdStore(state=>state.moveChildPrimitive);

    const data = {uuid,name,type,primitiveIds,transform};

    const fieldData = acceptLookup['node.primitive.hierarchical.program.'].primitiveIds;

    const ancestors = [
        {uuid:uuid,...fieldData},
        ...props.ancestors
    ];

    const [frame,focusItem,setFocusItem] = useGuiStore(state=>([
        state.frame,
        state.focusItem,
        state.setFocusItem
    ]));

    const focused = focusItem.uuid === uuid;

    // Code for handling the draggability of the program node itself
    const [{isDragging}, drag, preview] = useDrag({
        type: data.type,
        item: data,
        collect: monitor => ({
          isDragging: monitor.isDragging()
        })
    })

    const dragBlockStyles = {
        display:'inline-block',
        transform:`translate3d(${transform.x}px,${transform.y}px,0)`,
        opacity: isDragging ? 0.4 : 1,
        backgroundColor:
          blockStyles['node.primitive.hierarchical.program.'],
        minHeight: 30,
        minWidth: 250,
        borderRadius: 3,
        margin: 4,
        padding: 5,
        //position: 'relative',
        zIndex: focused ? 100 : 1
    };

    return (
        <div ref={preview} {...props} style={dragBlockStyles} className={focused?`focus-${frame}`:null}>
            <Row ref={drag} style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
                <span>
                    <Icon style={{marginLeft:4}} component={ContainerIcon} />{' '}{name}
                </span>
                <span style={{marginLeft:15}}>
                    <UnlockOutlined/>
                    <Button
                        type='text'
                        style={{marginLeft:2}}
                        onClick={() => setFocusItem('program', uuid)}
                        icon={<EllipsisOutlined />}
                    />
                </span>
            </Row>
            <NodeZone
              ancestors={ancestors}
              onDrop={(data) => moveChildPrimitive(data,uuid,0)}
              emptyMessage='No Actions'
              enabled={true}
            >
                {primitiveIds.map((id,idx)=>(
                    <ItemSortable key={id} id={id} idx={idx} ancestors={ancestors} itemType='primitive'/>
                ))}
            </NodeZone>
        </div>
    );
};
