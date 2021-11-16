import React, { useCallback, useState } from "react";
import { NodeZone } from "../NodeZone";
// import { ItemSortable } from "./Wrappers";
import { InputNumber, Row, Col, Button } from "antd";
import useStore from "../../../stores/Store";
import shallow from 'zustand/shallow';
import blockStyles from "../blockStyles";
import { ReactComponent as PrimitiveIcon } from '../../CustomIcons/Primitive.svg';
import Icon, { UnlockOutlined, LockOutlined, RightOutlined, EyeOutlined } from '@ant-design/icons';
import '../highlight.css';
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';

export const SkillCallBlock = ({ data, ancestors, context, dragDisabled, dropDisabled, listeners,attributes }) => {

  const [focused, skill, executable] = useStore(useCallback(state => {
    const executable = state.executablePrimitives[data.uuid] ? true : false;
    return [
      state.focusItem.uuid === data.uuid,
      state.data[data.parameters.skill],
      executable
    ]
  }, [data, context]), shallow);

  const [frame, setFocusItem, clearFocusItem, focusExists,
    setPrimitiveParameter] = useStore(
      (state) => [
        state.frame, state.setFocusItem,
        state.clearFocusItem, state.focusItem.type !== null,
      ], shallow)

  const handleStuff = listeners && attributes ? {
    ...listeners,
    ...attributes
  } : {}

  const unfocused = focusExists && !focused;

  const inDrawer = ancestors[0].uuid === 'drawer';
  const editingEnabled = !inDrawer && data.editable;

  const parameterAncestors = {
    machine: [
      { uuid: data.uuid, accepts: ['uuid-machine'] },
      ...ancestors
    ],
    location: [
      { uuid: data.uuid, accepts: ['uuid-location'] },
      ...ancestors
    ],
    trajectory: [
      { uuid: data.uuid, accepts: ['uuid-trajectory', 'trajectory'] },
      ...ancestors
    ],
    thing: [
      { uuid: data.uuid, accepts: ['uuid-thing'] },
      ...ancestors
    ]
  }

  // const [settingsRef, { height }] = useMeasure();
  const [settingsExpanded, setSettingsExpanded] = useState(false);
  const settingsStyle = useSpring({
    opacity: inDrawer || !settingsExpanded ? 0 : 1,
    scaleY: inDrawer || !settingsExpanded ? 0 : 1,
    config: config.stiff
  });
  const carrotStyle = useSpring({
    rotate: inDrawer || !settingsExpanded ? '0deg' : '90deg',
    config: config.wobbly
  });



  const styles = {
    backgroundColor:
      blockStyles[data.type],
    minHeight: 30,
    minWidth: 250,
    borderRadius: 3,
    fontSize: 14,
    padding: 5,
    position: 'relative',
    margin: 0,
    zIndex: focused ? 100 : 1
  };

  const fieldStyle = {
    borderRadius: 4,
    margin: '1pt 1pt 4pt 1pt',
    padding: 5,
    backgroundColor: "rgba(0,0,0,0.1)"
  }

  return (

    <div style={styles} className={focused ? `focus-${frame}` : null} onClick={(e) => { e.stopPropagation(); unfocused && clearFocusItem() }}>
      <Row wrap={false} style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
        <Row {...handleStuff} wrap={false} align='middle' span={17} style={{ textAlign: 'left', backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, minWidth: 200, maxWidth: 230, cursor: dragDisabled ? 'not-allowed' : 'grab', zIndex: 101 }}>
          <Icon style={{ marginLeft: 4, marginRight: 4 }} component={PrimitiveIcon} />
          {data.name}
        </Row>
        <Row wrap={false} align='middle' style={{ textAlign: 'end' }}>
          {executable && <Button type='text' icon={<EyeOutlined />} onClick={(e) => { e.stopPropagation(); setFocusItem('primitive', data.uuid) }} />}
          {editingEnabled ? <UnlockOutlined style={{ marginRight: 5, marginLeft: 5 }} /> : <LockOutlined style={{ marginRight: 5, marginLeft: 5 }} />}
          {/* <Button
                type='text'
                style={{marginLeft:2}}
                onClick={(e) => {e.stopPropagation();setFocusItem('program', uuid)}}
                icon={<EllipsisOutlined />}
            /> */}
        </Row>
      </Row>
      {skill.arguments.map(argument => (
        <Row align="middle" style={fieldStyle}>
          <Col flex={2} style={{ paddingRight: 5, textTransform: 'capitalize' }}>{argument.name}:</Col>
          <Col flex={3}>
            <NodeZone
              parentId={data.uuid}
              field={argument.uuid}
              ancestors={parameterAncestors[argument.type]}
              context={context}
              uuid={data.parameters[argument.uuid]}
              dragDisabled={!editingEnabled}
              dropDisabled={dropDisabled}
            />
          </Col>
        </Row>
      ))}
    </div>
  );
};
