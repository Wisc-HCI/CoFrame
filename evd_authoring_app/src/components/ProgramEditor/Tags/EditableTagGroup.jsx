// import { Button } from 'antd';
import React, {useCallback} from 'react';
import useStore from '../../../stores/Store';
import { typeToKey } from '../../../stores/helpers';

import { Menu, Button, Dropdown} from 'antd';
import { DownOutlined } from '@ant-design/icons';

import { EditableTag } from './EditableTag';
import { generateUuid } from '../../../stores/generateUuid';

import Icon from '@ant-design/icons';
import {ReactComponent as LocationIcon} from '../../CustomIcons/Location.svg';
import {ReactComponent as MachineIcon} from '../../CustomIcons/Gear.svg';
import {ReactComponent as ContainerIcon} from '../../CustomIcons/Container.svg';
import {ReactComponent as ThingIcon} from '../../CustomIcons/Thing.svg';

export const EditableTagGroup = (props) => {

  // const data = useEvdStore(useCallback(state=>state.data[typeToKey('parameter')]));
  const data = useStore(useCallback(state=>state.data[typeToKey('skill')][props.skill.uuid].parameters));

  const [createSkillParameter, deleteSkillParameter] = useStore(state=>[state.createSkillParameter, state.deleteSkillParameter]);

  const closeTag = tag => {
    deleteSkillParameter(props.skill.uuid, tag);
  };

  const handleMenuClick = e => {
    if (e.key === 'uuid-machine') {
      const item = {uuid: generateUuid('parameter'), name: '', itemType: 'machine', type: e.key, editting: true, closable: true}
      createSkillParameter(props.skill.uuid, item);
    }
    
  };

  const menu = (<Menu onClick={handleMenuClick}>
    <Menu.Item key="uuid-machine" icon={<Icon style={{marginRight:10}} component={MachineIcon}/>}>
      Machine
    </Menu.Item>
    <Menu.Item key="node.primitive.move-trajectory." icon={<Icon style={{marginRight:10}} component={ContainerIcon}/>}>
      Tracjectory
    </Menu.Item>
    <Menu.Item key="node.pose.waypoint.location." icon={<Icon style={{marginRight:10}} component={LocationIcon}/>}>
      Location
    </Menu.Item>
    <Menu.Item key="thing" icon={<Icon style={{marginRight:10}} component={ThingIcon}/>}>
      Thing
    </Menu.Item>
  </Menu>);

  return (
    <div>
      {Object.entries(data).map((obj) => {
        return <EditableTag key={obj[1].uuid} id={obj[0]} type={obj[1].type} parent={props.skill.uuid} itemType={obj[1].itemType} closefunc={closeTag}/>
      })}
      <Dropdown overlay={menu}>
        <Button>
          New Parameter <DownOutlined />
        </Button>
      </Dropdown>
    </div>
  );
}