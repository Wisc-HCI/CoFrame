import React from 'react';


import { RecipeEntryTile } from './RecipeEntryTile';
import { DeleteButton } from '../DeleteButton';
import { ElementTile } from '../ElementTile';
import { Card,List, Typography, Divider,Input,Button  } from 'antd';


export const MachineTile = (props) => {

    const {
        style,
        name,
        uuid,
        mesh,
        inputs,
        outputs,
        thingTypes,
        regions,
        deleteCallback,
        canDelete,
        canEdit
    } = props;

    return (
      <List.Item>

            <div style={{paddingLeft: '10px', paddingRight: '10px'}}>




                        Names:<Input onChange={() => {}} placeholder={name} styles={{ width: '400px'}} disabled={!canEdit}/>


                        <Button type = "primary" onClick={() => {}}  disabled={!canEdit} >
                          Edit Pose
                        </Button>


                        <DeleteButton type="Region" callback={() => { deleteCallback(uuid) }} disabled={!canDelete} />



                 <Divider/>

                Meshes:<Input onChange={() => {}} placeholder={mesh} styles={{width: '400px'}} disabled={!canEdit}/>





              Recipe:
              <Card>


                    Process Time:<Button
                    type= "primary"
                    style={{right :"-10px"}}

                        disabled={!canEdit}
                    >
                      0
                    </Button>


             <br/>
              <br/>
                Inputs:


                    {inputs.map(entry => (
                        <RecipeEntryTile
                            thingTypes={thingTypes}
                            selectedThing={entry.selectedThing}
                            quantity={entry.quantity}
                            regions={regions}
                            selectedRegion={entry.selectedRegion}
                            canEdit={canEdit}
                            canDelete={canDelete}
                        />
                    ))}


                        <Button   style={{right :"-10px"}} type= "primary" text="Add Input" onClick={() => {
                            inputs.push({

                            })
                        }} disabled={!canEdit}>
                          Add Input
                        </Button>



                <br/>
                <br/>
                Outputs:


                    {outputs.map(entry => (
                        <RecipeEntryTile
                            thingTypes={thingTypes}
                            selectedThing={entry.selectedThing}
                            quantity={entry.quantity}
                            regions={regions}
                            selectedRegion={entry.selectedRegion}
                            canEdit={canEdit}
                            canDelete={canDelete}
                        />
                    ))}


                        <Button style={{right: "-10px"}}onClick={() => {}} disabled={!canEdit}>
                          Add Output
                        </Button>

                </Card>
                </div>

          </List.Item>



    );
};
