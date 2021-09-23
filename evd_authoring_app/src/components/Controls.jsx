import React from 'react';

import { CaretRightFilled, PauseOutlined, RollbackOutlined, FullscreenExitOutlined, FullscreenOutlined, EllipsisOutlined } from '@ant-design/icons';
import { Button, Space, Tooltip, Popover, Checkbox, Col, Row } from 'antd';

import useStore from '../stores/Store';
import shallow from 'zustand/shallow';

export function Controls() {

  const [
    simMode, setSimMode,
    collisionsVisible, setCollisionsVisible,
    occupancyVisible, setOccupancyVisible
  ] = useStore(state => ([state.simMode, state.setSimMode, state.collisionsVisible, state.setCollisionsVisible, state.occupancyVisible, state.setOccupancyVisible]), shallow)

  return (

    <Space>
      <Tooltip placement="top" title={simMode === 'default' ? "Expand" : "Shrink"}>
        <Button
          type="outline"
          icon={simMode === 'default' ? <FullscreenOutlined /> : <FullscreenExitOutlined />}
          onClick={() => setSimMode(simMode === 'default' ? 'expanded' : 'default')}
        />
      </Tooltip>
      <Tooltip placement="top" title="Play">
        <Button
          type="outline"
          icon={<CaretRightFilled />}
          onClick={() => console.log('PLAY')}
        />
      </Tooltip>
      <Tooltip placement="top" title="Pause">
        <Button
          type="outline"
          icon={<PauseOutlined />}
          onClick={() => console.log('PAUSE')}
        />
      </Tooltip>
      <Tooltip placement="top" title="Refresh">
        <Button
          type="outline"
          icon={<RollbackOutlined />}
          onClick={() => console.log('REFRESH')}
        />
      </Tooltip>
      <Popover
        title="Configure"
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
      </Popover>

    </Space>
  );
}
