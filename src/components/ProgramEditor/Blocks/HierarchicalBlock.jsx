import React, { useCallback, useState } from 'react';
import { Badge, Button, Row, Input } from 'antd';
import Icon, { RightOutlined, EditOutlined, SaveOutlined, EyeOutlined } from '@ant-design/icons';
import { NodeList } from '../NodeList';
import useStore from '../../../stores/Store';
import shallow from 'zustand/shallow';
import blockStyles from '../blockStyles';
import { ReactComponent as ContainerIcon } from '../../CustomIcons/Container.svg';
import { acceptLookup } from '../acceptLookup';
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
import '../highlight.css';

export const HierarchicalBlock = ({ data, ancestors, context, dragDisabled, dropDisabled, listeners,attributes }) => {

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
  
  const editingEnabled = !inDrawer && data.editable;

  const fieldData = acceptLookup['hierarchical'].children;

  const primitiveAncestors = [
    { uuid: data.uuid, ...fieldData },
    ...ancestors
  ]

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
    zIndex: focused ? 100 : 1
  };

  const handleStuff = listeners && attributes ? {
    ...listeners,
    ...attributes
  } : {}

  return (
    <div style={dragBlockStyles} className={focused ? `focus-${frame}` : null}>
      <Row wrap={false} style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
        <Row {...handleStuff} wrap={false} align='middle' style={{ boxShadow: editing?'inset 0px 0px 2px 1px #ffffff':null, backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start', flex: 1, minWidth: 130, maxWidth: 200, cursor: dragDisabled ? "not-allowed" : "grab",zIndex:101, marginRight:5, height: 32 }}>
          <Icon style={{ marginLeft: 4 }} component={ContainerIcon} />
          <Input style={{maxWidth: 200, color:'white',cursor: editing ? 'text' : dragDisabled ? "not-allowed" : "grab"}} bordered={false} disabled={!editing} value={data.name} onChange={(e)=>setItemProperty('primitive', data.uuid, 'name', e.target.value)}/> 
        </Row>
        <Row wrap={false} align='middle' style={{ textAlign: 'end' }}>
          {!inDrawer && <Button type='text' onClick={() => setExpanded(!expanded)} icon={<animated.div style={carrotStyle}><RightOutlined/></animated.div>} style={{zIndex:200}}/>}
          {editingEnabled && <Button type='text' onClick={() => setEditing(!editing)} icon={editing ? <SaveOutlined/> : <EditOutlined/>}/>}
          {executable && <Button type='text' icon={<EyeOutlined/>} onClick={(e) => {e.stopPropagation();setFocusItem('primitive', data.uuid)}}/>}
          <Badge count={data.children.length} showZero={true} style={{backgroundColor:'rgba(0,0,0,0.3)',marginRight:5, marginLeft:5}}/>
        </Row>
      </Row>
      <animated.div style={nodeListStyle}>
        <NodeList ancestors={primitiveAncestors} uuids={data.children} field='children' context={context} dragDisabled={!data.editable || dragDisabled} dropDisabled={dropDisabled}/>
      </animated.div>
    </div>
  )
};