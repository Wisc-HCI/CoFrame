import React from 'react';
//import { List, Row, Button } from 'antd';
import { Accordion, Button,AccordionPanel,Box } from 'grommet';
import { InputOutputComponent } from './InputOutputComponent'
import Item from 'antd/lib/list/Item';



export const MachineInOutTypeDetail = (props) => {
  // let comKeys = null;
  // if (props.input){
  //    comKeys = Object.keys(props.machine.inputs);
  // }else {
  //    comKeys = Object.keys(props.machine.outputs);
  // }

  const type = {
    something: "sefsef"
  }

  const input = [
    {
      thingType: "thingType-98892bd01c1911ecbe2600155d1a70a2",
      relativeTo: "machine-f74ac8e4-d7f1-4492-8e60-bd2ae6644e5c",
      position: {
        x: 0,
        y: 0,
        z: 0
      },
      orientation: {
        x: 0,
        y: 0,
        z: 0,
        w: 1
      }
    },
    // {
    //   "thingType": "thingType-98892a7c1c1911ecbe2600155d1a70a2",
    //   "relativeTo": "machine-f74ac8e4-d7f1-4492-8e60-bd2ae6644e5c",
    //   "position": {
    //     "x": 0,
    //     "y": 0,
    //     "z": 0
    //   },
    //   "orientation": {
    //     "x": 0,
    //     "y": 0,
    //     "z": 0,
    //     "w": 1
    //   }
    // },
    // {
    //   "thingType": "thingType-98892b3a1c1911ecbe2600155d1a70a2",
    //   "relativeTo": "machine-f74ac8e4-d7f1-4492-8e60-bd2ae6644e5c",
    //   "position": {
    //     "x": 0,
    //     "y": 0,
    //     "z": 0
    //   },
    //   "orientation": {
    //     "x": 0,
    //     "y": 0,
    //     "z": 0,
    //     "w": 1
    //   }
    // }
  ]

  // function getPanel(){

  //   return (<>
  //     {input.forEach((item) => {
  //       item.thingType
  //     })}

  //   </>)
  // }
   return (
    <>
      <br />
      {/* <List 
        header={<Row align='middle' justify='space-between'>{props.input ? 'Inputs' : 'Outputs'}<Button>Add</Button></Row>}
        split={false}
        bordered 
        dataSource={comKeys} 
        renderItem = {(uuid) => (
          <List.Item> 
            <InputOutputComponent typeuuid={uuid} data = {props.machine[props.input ? 'inputs' : 'outputs'][uuid]}/>
          </List.Item>
        )}>

      </List> */}
      asdfasdf
      <Accordion>
        {
          input.forEach((item) => {
            <AccordionPanel label = {item.thingType}>
              <Box pad="medium" background="light-2">
                <Text>{item.position.x}</Text>
                <Text>{item.position.y}</Text>
                <Text>{item.position.z}</Text>
              </Box>
            </AccordionPanel>

          })
        }



      </Accordion>
    </>)



}
