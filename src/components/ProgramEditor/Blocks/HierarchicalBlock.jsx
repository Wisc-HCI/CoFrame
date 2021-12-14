import React, { useCallback, useState } from 'react';
import { Badge, Button, Row, Input } from 'antd';
import Icon, { RightOutlined, EditOutlined, SaveOutlined, EyeOutlined } from '@ant-design/icons';
import { NodeList } from '../NodeList';
import { Base } from './Base';
import useStore from '../../../stores/Store';
import shallow from 'zustand/shallow';
import blockStyles from '../blockStyles';
import { ReactComponent as ContainerIcon } from '../../CustomIcons/Container.svg';
import { acceptLookup } from '../acceptLookup';
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
import '../highlight.css';

export const HierarchicalBlock = ({ data, dragHandle, ancestors, context, dragDisabled, dropDisabled, style }) => {

  const [frame, focusItem, setFocusItem, setItemProperty] = useStore(state => (
      [state.frame, state.focusItem, state.setFocusItem, state.setItemProperty]),shallow);

  const executable = useStore(useCallback((state) => {
    return state.executablePrimitives[data.uuid] ? true : false;
  }, [data]),shallow)

  const inDrawer = ancestors[0].uuid === 'drawer';

  const [editing, setEditing] = useState(false);
  const [expanded, setExpanded] = useState(true);
  const nodeListStyle = useSpring({ 
    scaleY: expanded ? 1 : 0,
    opacity: expanded ? 1 : 0,
    config: config.stiff 
  });
  const carrotStyle = useSpring({
    rotate: inDrawer || !expanded ? '0deg' : '90deg',
    config: config.wobbly
  });

  const focused = focusItem.uuid === data.uuid;
  
  const editingEnabled = !inDrawer && !data.readonly;

  const fieldData = acceptLookup['hierarchical'].children;

  const primitiveAncestors = [
    { uuid: data.uuid, ...fieldData },
    ...ancestors
  ]

  console.log({children:data.children, dragDisabled: data.readonly || dragDisabled})

  const dragBlockStyles = {
    backgroundColor:
      blockStyles[data.type],
    minHeight: 30,
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

  return (
    <Base
      dragHandle={dragHandle}
      dragDisabled={dragDisabled}
      focused={focused}
      locked={data.readonly || inDrawer}
      name={data.name}
      nameEditable={editing}
      onNameChange={(v)=>setItemProperty(data.uuid, 'name', v)}
      type={data.type}
      extra={<>
        {!inDrawer && <Button type='text' onClick={() => setExpanded(!expanded)} icon={<animated.div style={carrotStyle}><RightOutlined/></animated.div>} style={{zIndex:200}}/>}
        {editingEnabled && <Button type='text' onClick={() => setEditing(!editing)} icon={editing ? <SaveOutlined/> : <EditOutlined/>}/>}
        {executable && <Button type='text' icon={<EyeOutlined/>} onClick={(e) => {e.stopPropagation();setFocusItem('data', data.uuid)}}/>}
        <Badge count={data.children.length} showZero={true} style={{backgroundColor:'rgba(0,0,0,0.3)',marginRight:5, marginLeft:5}}/>
      </>}
    >
      <animated.div style={nodeListStyle}>
        {expanded && (
          <NodeList ancestors={primitiveAncestors} uuids={data.children} field='children' context={context} dragDisabled={data.readonly || dragDisabled} dropDisabled={dropDisabled}/>
        )}
      </animated.div>
    </Base>
  )
};