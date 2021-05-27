import React from 'react';

import { Toggle } from '@fluentui/react/lib/Toggle';
import {CaretRightFilled,PauseOutlined,RollbackOutlined  } from '@ant-design/icons';
import { Button, Radio , Space,Tooltip,Switch} from 'antd';



export class Controls extends React.Component {

  constructor(props) {
    super(props);

    this.onPauseClicked = this.onPauseClicked.bind(this);
    this.onPlayClicked = this.onPlayClicked.bind(this);
    this.onResetClicked = this.onResetClicked.bind(this);
  }

  onPauseClicked() {
    console.log('On pause clicked');
  }

  onPlayClicked() {
    console.log('On play clicked');
  }

  onResetClicked() {
    console.log('On reset clicked');
  }

  render() {

    const {
        width,
        height
    } = this.props;

    return (

                        <div>

                      
                        <Space>
                         <Tooltip placement="topLeft" title="Play">
                        <Button ghost
                            type = "primary"
                            shape = "circle"

                            icon= {<CaretRightFilled />}
                            onClick={() => this.onPlayClicked}
                        />
                        </Tooltip>
                        <Tooltip placement="topLeft" title="Pause">
                        <Button ghost
                        type = "primary"
                        shape = "circle"
                        icon= {<PauseOutlined />}
                        onClick={() => this.onPauseClicked}
                        />
                        </Tooltip>
                        <Tooltip placement="topLeft" title="Refresh">
                        <Button ghost
                        type = "primary"
                        shape = "circle"
                        icon= {<RollbackOutlined />}
                        onClick={() => this.onResetClicked}
                        />
                        </Tooltip>





                        </Space>







                        </div>




    );
  }
}
