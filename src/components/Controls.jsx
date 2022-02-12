import React from 'react';

import { CaretRightFilled, PauseOutlined, RollbackOutlined, FullscreenExitOutlined, FullscreenOutlined, EllipsisOutlined } from '@ant-design/icons';
// import { Button, Space, Tooltip, Popover, Checkbox, Col, Row } from 'antd';
import { Button, Box} from 'grommet';
import { TipContent } from './TipContent';

import useStore from '../stores/Store';
import shallow from 'zustand/shallow';

export function Controls() {

  const [
    simMode, setSimMode,
    collisionsVisible, setCollisionsVisible,
    occupancyVisible, setOccupancyVisible
  ] = useStore(state => ([state.simMode, state.setSimMode, state.collisionsVisible, state.setCollisionsVisible, state.occupancyVisible, state.setOccupancyVisible]), shallow)

  return (

    <Box direction='row' margin={{right:'small'}}>
      
        <Button
          tip={{
            content: <TipContent message={simMode === 'default' ? 'Expand' : 'Shrink'} />,
            plain: true,
            dropProps: {
                align: { bottom: 'top' }
            }
          }}
          icon={simMode === 'default' ? <FullscreenOutlined /> : <FullscreenExitOutlined />}
          onClick={() => setSimMode(simMode === 'default' ? 'expanded' : 'default')}
        />
      
      
        <Button
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
        />
      
      {/* <Popover
        placement="bottomRight"
        content={
          <Col>
            <Row>
              <Checkbox
                checked={collisionsVisible}
                onChange={() => setCollisionsVisible(!collisionsVisible)}
              >
                Show Collisions
              </Checkbox>
            </Row>
            <Row>
              <Checkbox
                checked={occupancyVisible}
                onChange={() => setOccupancyVisible(!occupancyVisible)}
              >
                Show Human Occupancy
              </Checkbox>
            </Row>
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
