import React from 'react';
import useStore from '../../stores/Store';
import { NumberInput } from '../NumberInput';
import { Box, DropButton, Text } from "grommet";
import { FormEdit } from "grommet-icons";

function PositionInput(props) {
  let minMax = [-10, 10];
  const updateItemPositionProperty = useStore(state => state.updateItemPositionProperty);
  
  return (

    <div style={{ display: 'flex', flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center' }}>
      <div>
        <Text style={{ color: 'rgba(255, 255, 255, 0.85)' }} >Position:</Text>
      </div>

      <div >
        <DropButton
          primary
          label="Edit"
          icon={<FormEdit />}
          dropAlign={{ right: 'right', top: "bottom" }}
          dropProps={{ elevation: 'none', round: "xsmall" }}

          dropContent={
            <Box round="xsmall" background="grey" border={{ color: 'white', size: 'xsmall' }} width="small" direction='column' elevation="none" pad="xsmall" justify='center'>
              <Box direction='row' elevation="none" pad="xsmall" justify='center' width="small">
                <Text weight="bolder" style={{ color: "red", paddingRight: "7%" }}>X</Text>
                <NumberInput
                  min={minMax[0]}
                  max={minMax[1]}
                  value={props.position.x}
                  onChange={(value) => updateItemPositionProperty(props.itemID, 'x', value)}

                />
              </Box>

              <Box direction='row' elevation="none" pad="xsmall" justify='center' width="small">
                <Text weight="bolder" style={{ color: "lime", paddingRight: "7%" }}>Y</Text>
                <NumberInput
                  min={minMax[0]}
                  max={minMax[1]}
                  value={props.position.y}
                  onChange={(value) => updateItemPositionProperty(props.itemID, 'y', value)}

                />
              </Box>

              <Box direction='row' elevation="none" pad="xsmall" justify='center' width="small">
                <Text weight="bolder" style={{ color: "blue", paddingRight: "7%" }}>Z</Text>
                <NumberInput
                  min={minMax[0]}
                  max={minMax[1]}
                  value={props.position.z}
                  onChange={(value) => updateItemPositionProperty(props.itemID, 'z', value)}

                />
              </Box>

            </Box>
          }
        />
      </div>
    </div>
  )


}
export default (PositionInput);
