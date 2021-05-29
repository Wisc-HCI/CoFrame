import React from 'react';




import { DeleteButton } from '../DeleteButton';
import { ElementTile } from '../ElementTile';
import { Card,List, Typography, Divider,Input,Button,Dropdown ,Menu } from 'antd';
import { DownOutlined } from '@ant-design/icons';


export const RegionTile = (props) => {

    const { style, name, uuid, deleteCallback, canDelete, canEdit } = props;



    const selectedOption = "orientation";
    const menu = (
  <Menu>
    <Menu.Item key="orientation" >
    Orientation-Uncertainty-Only
    </Menu.Item>
    <Menu.Item key="box">
      Box-Pose-Uncertainty
    </Menu.Item>
    <Menu.Item key="sphere" >
    Sphere-Pose-Uncertainty
    </Menu.Item>
  </Menu>
);

    return (
        <List.Item>


                    Name:<Input onChange={() => {}} placeholder={name} styles={{width: '400px'}}/>
                    <br/>
                    <br/>



                    Type:<Dropdown

                        overlay={menu}



                    >
                    <Button   style={{right: "-10px" }}>
                       Dropdown <DownOutlined />
                     </Button>

                    </Dropdown>


                    <Button type='primary'onClick={() => {}}  disabled={!canEdit}>
                      Edit Pose
                    </Button>

                    <DeleteButton type="Region" callback={() => { deleteCallback(uuid) }} disabled={!canDelete} />


        </List.Item>
    );
};
