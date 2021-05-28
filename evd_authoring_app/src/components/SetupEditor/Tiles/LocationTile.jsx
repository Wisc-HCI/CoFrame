import React from 'react';



import { DeleteButton } from '../DeleteButton';
import { ElementTile } from '../ElementTile';
import { Card,List, Typography, Divider,Input,Button  } from 'antd';


export const LocationTile = (props) => {

    const { style, name, uuid, deleteCallback, canDelete, canEdit } = props;


    return (
        <List.Item  >


                    <Input label="Name:" placeholder={name}/>


                    <Button style={{width : '100px'}}type="primary" text="Edit Pose" disabled={!canEdit}>
                      Edit Pose
                    </Button>


                    <DeleteButton type="Waypoint" callback={() => { deleteCallback(uuid) }} disabled={!canDelete} />


        </List.Item>
    );
};
