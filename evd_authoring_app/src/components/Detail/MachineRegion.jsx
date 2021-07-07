import React from 'react';


import { Button, Divider, Input } from 'antd';

export const MachineRegion = (props) => {


  return (
    <>

      <Divider orientation="left" style={{ color: 'white', borderTopColor: 'rgba(255,255,255,0.12)', lineHeight: '1.5715px', paddingTop: '20px', paddingBottom: '5px' }}>
        Input:
      </Divider>
      <br />
      <br />
      <br />
      <div style={{ color: 'white', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        Quantity :
        <Input style={{ width: '20%' }} value={props.inputQuantity} />
      </div>
      <br />
      <div style={{ color: 'white', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        Region:
        <Button style={{ width: '20%' }} >
          Edit
        </Button>


      </div>




      <Divider orientation="left" style={{ color: 'white', borderTopColor: 'rgba(255,255,255,0.12)', lineHeight: '1.5715px', paddingTop: '20px', paddingBottom: '5px' }}>
        Output:
      </Divider>
      <br />
      <b style={{ color: 'white' }}> region.name </b>
      <br />
      <br />
      <div style={{ color: 'white', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        Quantity :
        <Input style={{ width: '20%' }} value={props.outputQuantity} />
      </div>
      <br />
      <div style={{ color: 'white', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        Region:
        <Button style={{ width: '20%' }} >
          Edit
        </Button>

      </div>
      <Divider />
    </>





  )
}
