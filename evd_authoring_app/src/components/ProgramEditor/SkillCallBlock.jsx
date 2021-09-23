import React, { useCallback } from "react";
import { NodeZone } from "./NodeZone";
// import { ItemSortable } from "./Wrappers";
import { Row, Col } from "antd";
import useStore from "../../stores/Store";
import shallow from 'zustand/shallow';
import blockStyles from "./blockStyles";
import { ReactComponent as SkillIcon } from '../CustomIcons/Skill.svg';
import Icon, { UnlockOutlined, LockOutlined } from '@ant-design/icons';
import { TrajectoryBlock } from "./TrajectoryBlock";
import './highlight.css';
import { UUIDBlock } from "./UUIDBlock";
import { useDrag } from "react-dnd";

const TYPE_LOOKUP = {
  'node.machine.': {store:'machines',type:'machine'},
  'node.pose.thing.': {store:'placeholders',type:'thing'},
  'node.pose.waypoint.': {store:'waypoints',type:'waypoint'},
  'node.pose.waypoint.location.': {store:'locations',type:'location'},
  'node.trajectory.': {store:'trajectories',type:'trajectory'}
}

const PRETTY_LOOKUP = {
  'node.machine.': 'Machine',
  'node.pose.thing.': 'Thing',
  'node.pose.waypoint.': 'Waypoint',
  'node.pose.waypoint.location.': 'Location',
  'node.trajectory.': 'Trajectory'
}

export const SkillCallBlock = ({
  staticData, uuid, parentData, onDelete, dragBehavior,
  dragDisabled, ancestors, context, idx, after,
}) => {

  const [focused, data, skill, parameters] = useStore(useCallback(state => {
    let parameterValues = [];

    /*
    {
        type: 'node.skill-argument.',
        // The human-readable name of this argument
        name: 'This Machine',
        // If the uuid is found in any fields for the children primitives' parameters, replace it with the corresponding value. 
        // This also serves as the key for any corresponding skill-call's parameters.
        uuid: 'skill-arg-uuid-0', 
        editable: true,
        deleteable: true,
        description: '',
        // The type of the argument
        parameter_type: 'node.machine.',
        is_list: false
    }
    */
    const data = staticData ? staticData : state.data.primitives[uuid];
    const skill = state.data.skills[data.parameters.skill_uuid];

    skill.arguments.forEach(argument => {
      const callVal = data.parameters[argument.uuid];
      if (argument.parameter_type !== 'node.pose.thing.') {
        if (callVal && state.data[TYPE_LOOKUP[argument.parameter_type].store][callVal]) {
          parameterValues.push({ contents: state.data[TYPE_LOOKUP[argument.parameter_type].store][callVal], argument, real:true })
        } else if (callVal && context[callVal]) {
          parameterValues.push({ contents: { uuid: callVal, ...context[callVal] }, argument })
        } else {
          parameterValues.push({ contents: null, argument })
        }
      } else if (argument.parameter_type === 'node.pose.thing.') {
        if (callVal && state.data.placeholders[callVal]) {
          parameterValues.push({ contents: state.data.placeholders[callVal], argument })
        } else if (callVal && context[callVal]) {
          parameterValues.push({ contents: { uuid: callVal, pending_node: context[callVal] }, argument })
        } else {
          parameterValues.push({ contents: null, argument })
        }
      }
    })

    return [
      state.focusItem.uuid === uuid,
      data,
      skill,
      parameterValues
    ]
  }, [staticData, uuid, context]),shallow);

  const [frame, clearFocusItem, focusExists,
    setPrimitiveParameter, moveTrajectoryBlock,
    deletePrimitiveTrajectory] = useStore(
      (state) => [state.frame, state.clearFocusItem, state.focusItem.type !== null,
      state.setPrimitiveParameter, state.moveTrajectoryBlock, state.deletePrimitiveTrajectory],shallow)

  const unfocused = focusExists && !focused;

  const inDrawer = ancestors[0].uuid === 'drawer';
  const editingEnabled = !inDrawer && data.editable;

  const [{ isDragging }, drag, preview] = useDrag(() => ({
    type: data.type,
    item: { ...data, parentData, dragBehavior, onDelete, idx },
    options: { dragEffect: dragBehavior },
    canDrag: (_) => !dragDisabled,
    collect: monitor => ({
      isDragging: monitor.isDragging()
    })
  }))


  const parameterAncestors = {
    'node.machine.': [
      { uuid: data.uuid, accepts: ['uuid-machine'] },
      ...ancestors
    ],
    'node.pose.waypoint.location.': [
      { uuid: data.uuid, accepts: ['uuid-location'] },
      ...ancestors
    ],
    'node.pose.waypoint.': [
      { uuid: data.uuid, accepts: ['uuid-waypoint'] },
      ...ancestors
    ],
    'node.trajectory.': [
      { uuid: data.uuid, accepts: ['uuid-trajectory', 'node.trajectory.'] },
      ...ancestors
    ],
    'node.pose.thing.': [
      { uuid: data.uuid, accepts: ['uuid-thing'] },
      ...ancestors
    ]
  }

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

  const parameterDrop = (dropData, parameter) => {
    if (dropData.type === 'node.trajectory.') {
      moveTrajectoryBlock(dropData, uuid, parameter)
    } else if (dropData.parentData.uuid === uuid && dropData.dragBehavior === 'move') {
      setPrimitiveParameter('primitive', uuid, dropData.parentData.field, null);
      setPrimitiveParameter('primitive', uuid, parameter, dropData.uuid);
    } else {
      setPrimitiveParameter('primitive', uuid, parameter, dropData.uuid);
    }
  }

  return (
    <div hidden={isDragging && dragBehavior === 'move'}>
      <div ref={preview} style={styles} className={focused ? `focus-${frame}` : null} onClick={(e) => { e.stopPropagation(); unfocused && clearFocusItem() }}>
        <Row style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
          <Col ref={dragDisabled ? null : drag} span={17} style={{ textAlign: 'left', backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, cursor: dragDisabled ? 'not-allowed' : 'grab', zIndex: 101 }}>
            <Icon style={{ marginLeft: 4 }} component={SkillIcon} />{' Execute: '}{skill.name}
          </Col>
          <Col span={6} offset={1} style={{ textAlign: 'end' }}>
          {editingEnabled ? <UnlockOutlined style={{marginRight:5}}/> : <LockOutlined style={{marginRight:5}}/>}
            {/* <Button
                  type='text'
                  style={{marginLeft:2}}
                  onClick={(e) => {e.stopPropagation();setFocusItem('program', uuid)}}
                  icon={<EllipsisOutlined />}
              /> */}
          </Col>
        </Row>
        {parameters.map((argInfo,i) => (
          <Row key={i} align="middle" style={fieldStyle}>
            <Col flex={2} style={{ paddingRight: 5 }}>{argInfo.argument.name}:</Col>
            <Col flex={3}>
              <NodeZone
                ancestors={parameterAncestors[argInfo.argument.parameter_type]}
                context={context}
                onDrop={(dropData) => parameterDrop(dropData, argInfo.argument.uuid)}
                emptyMessage={`No ${PRETTY_LOOKUP[argInfo.argument.parameter_type]}`}
                dropDisabled={!editingEnabled}
              >
                {argInfo.contents && (argInfo.argument.parameter_type !== 'node.trajectory.' || !argInfo.real) ? (
                  <UUIDBlock
                    id={argInfo.contents.uuid}
                    idx={0}
                    dragBehavior='move'
                    hoverBehavior='replace'
                    ancestors={parameterAncestors[argInfo.argument.parameter_type]}
                    context={context}
                    parentData={{ type: 'primitive', uuid, field: argInfo.argument.uuid }}
                    data={{ ...argInfo.contents, itemType: TYPE_LOOKUP[argInfo.argument.parameter_type].type, type: `uuid-${TYPE_LOOKUP[argInfo.argument.parameter_type].type}` }}
                    onDelete={editingEnabled ? (_) => setPrimitiveParameter('primitive', uuid, argInfo.argument.uuid, null) : null}
                    onDrop={(dropData) => parameterDrop(dropData, argInfo.argument.uuid)}
                    dragDisabled={!editingEnabled}
                    dropDisabled={!editingEnabled}
                  />
                ) : argInfo.contents && argInfo.argument.parameter_type === 'node.trajectory.' && argInfo.real ? (
                  <TrajectoryBlock
                    uuid={argInfo.contents.uuid}
                    idx={0}
                    parentData={{ type: 'primitive', uuid, field: argInfo.argument.uuid }}
                    ancestors={parameterAncestors[argInfo.argument.parameter_type]}
                    dragBehavior='move'
                    dragDisabled={!editingEnabled}
                    onDelete={(_) => deletePrimitiveTrajectory(uuid, argInfo.argument.uuid, argInfo.contents.uuid)}
                    context={context}
                  />
                ) : null}
              </NodeZone>
            </Col>
          </Row>
        ))}
      </div>
      {!(isDragging && dragBehavior === 'move') && after}
    </div>
  );
};
