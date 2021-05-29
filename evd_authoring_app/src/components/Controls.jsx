import React from 'react';

import {CaretRightFilled,PauseOutlined,RollbackOutlined  } from '@ant-design/icons';
import { Button, Space, Tooltip } from 'antd';

export function Controls() {

  return (

    <Space>
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
