import React from 'react';

import {CaretRightFilled,PauseOutlined,RollbackOutlined,FullscreenExitOutlined,FullscreenOutlined } from '@ant-design/icons';
import { Button, Space, Tooltip } from 'antd';

import useGuiStore from '../stores/GuiStore';

export function Controls() {

  const [simMode,setSimMode] = useGuiStore(state=>([state.simMode, state.setSimMode]))

  return (

    <Space>
      <Tooltip placement="top" title={simMode === 'default' ? "Expand" : "Shrink"}>
        <Button 
          type="outline"
          icon= {simMode === 'default' ? <FullscreenOutlined /> : <FullscreenExitOutlined />}
          onClick={() => setSimMode(simMode === 'default' ? 'expanded' : 'default')}
        />
      </Tooltip>
      <Tooltip placement="top" title="Play">
        <Button 
          type="outline"
          icon= {<CaretRightFilled />}
          onClick={() => console.log('PLAY')}
        />
      </Tooltip>
      <Tooltip placement="top" title="Pause">
        <Button 
          type="outline"
          icon= {<PauseOutlined />}
          onClick={() => console.log('PAUSE')}
        />
      </Tooltip>
      <Tooltip placement="top" title="Refresh">
        <Button 
          type="outline"
          icon= {<RollbackOutlined />}
          onClick={() => console.log('REFRESH')}
        />
      </Tooltip>
    </Space>
  );
}
