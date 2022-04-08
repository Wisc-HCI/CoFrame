import React from 'react';

// import { CaretRightFilled, PauseOutlined, RollbackOutlined, FullscreenExitOutlined, FullscreenOutlined, EllipsisOutlined } from '@ant-design/icons';
import { FiMinimize, FiMaximize, FiMoreHorizontal } from 'react-icons/fi';
import { Toggle } from './Toggle';
// import { Button, Space, Tooltip, Popover, Checkbox, Col, Row } from 'antd';
import { Box, DropButton, Button, Text } from "grommet";
import { TipContent } from './TipContent';

import useStore from '../stores/Store';
import shallow from 'zustand/shallow';

export function Controls() {

  const [
    primaryColor,
    viewMode, setViewMode,
    collisionsVisible, setCollisionsVisible,
    occupancyVisible, setOccupancyVisible,
    tfVisible, setTfVisible
  ] = useStore(state => ([state.primaryColor, state.viewMode, state.setViewMode, state.collisionsVisible, state.setCollisionsVisible, state.occupancyVisible, state.setOccupancyVisible, state.tfVisible, state.setTfVisible]), shallow)

  return (

    <Box direction='row' margin={{ right: 'small' }} align='center' alignContent='center'>

      <Button
        tip={{
          content: <TipContent message={viewMode === 'default' ? 'Expand' : 'Shrink'} inverted />,
          plain: true,
          dropProps: {
            align: { top: 'bottom' }
          }
        }}
        icon={viewMode === 'default' ? <FiMaximize /> : <FiMinimize />}
        onClick={() => setViewMode(viewMode === 'default' ? 'sim' : 'default')}
      />


      {/* <Button
          tip={{
            content: <TipContent message='Play' />,
            plain: true,
            dropProps: {
                align: { bottom: 'top' }
            }
          }}
          icon={<CaretRightFilled />}
          onClick={() => console.log('PLAY')}
        />
      
        <Button
          tip={{
            content: <TipContent message='Pause' />,
            plain: true,
            dropProps: {
                align: { bottom: 'top' }
            }
          }}
          icon={<PauseOutlined />}
          onClick={() => console.log('PAUSE')}
        />
     
        <Button
          tip={{
            content: <TipContent message='Refresh' />,
            plain: true,
            dropProps: {
                align: { bottom: 'top' }
            }
          }}
          icon={<RollbackOutlined />}
          onClick={() => console.log('REFRESH')}
        /> */}

      <DropButton
        focusIndicator={false}
        hoverIndicator={false}
        dropContent={
          <Box round='xsmall' background='grey' direction="column" align='start' border={{ color: 'lightgrey' }} pad='small' gap='small'>
            <Box direction='row' justify='between' flex gap='xsmall' onClick={() => setCollisionsVisible(!collisionsVisible)} focusIndicator={false}>
              <Toggle size='small' selected={collisionsVisible} backgroundColor={primaryColor}/>
              Show Collisions
            </Box>
            <Box direction='row' justify='between' flex gap='xsmall' onClick={() => setOccupancyVisible(!occupancyVisible)} focusIndicator={false}>
            <Toggle size='small' selected={occupancyVisible} backgroundColor={primaryColor}/>
              Show Human Occupancy
            </Box>
            <Box direction='row' justify='between' flex gap='xsmall' onClick={() => setTfVisible(!tfVisible)} focusIndicator={false}>
            <Toggle size='small' selected={tfVisible} backgroundColor={primaryColor}/>
              Show TFs
            </Box>
          </Box>
        }
        dropProps={{ align: { top: 'bottom' }, elevation: 'none', background: 'none' }}
      >
        <Box margin='small' >
          <FiMoreHorizontal />
        </Box>
        
      </DropButton>

      {/* <Popover
        placement="bottomRight"
        content={
          <Col>
            
          </Col>
        }
      >
        <Button
          type="outline"
          icon={<EllipsisOutlined />}
        />
      </Popover> */}

    </Box>
  );
}
