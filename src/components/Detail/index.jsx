import { React } from 'react';
import { LocationDetail } from './LocationDetail';
import { MachineProcessList } from './MachineDetail';
import { ThingDetail } from './ThingDetail';
import { WaypointDetail } from './WaypointDetail';
import { ProcessDetail } from './ProcessDetail';
import { InputOutputDetail } from './InputOutputDetail';

import { TextArea, Text, Box, TextInput, Button, Layer, DropButton } from 'grommet';

import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';
import { FiTrash, FiX } from 'react-icons/fi';

const DETAIL_TYPES = ['machineType','inputOutputType','processType','locationType','waypointType','thingType']

export const Detail = (_) => {

  const {
    item,
    objectTypeInfo
  } = useStore(state => {
    let item = null;
    state.focus.slice().reverse().some(v => {
      if (state.programData[v] && state.activeFocus === v && DETAIL_TYPES.includes(state.programData[v].type)) {
        item = state.programData[v]
        return true
      } else {
        return false
      }
    })
    return {
      item,
      objectTypeInfo: item?.type ? state.programSpec.objectTypes[state.programData[item?.id].type] : null
    }
  }, shallow);

  const addFocusItem = useStore(state => state.addFocusItem);
  const clearFocus = useStore(state => state.clearFocus);

  // const [processItem, setProcessItem] = useState(null);
  // const [inputOutputItem, setInputOutputItem] = useState(null);
  // const [position, setPosition] = useState(null);
  // const [rotation, setRotation] = useState(null);
  console.log('BLOCK DATA',objectTypeInfo)
  const objectColor = objectTypeInfo?.instanceBlock?.color 
    ? objectTypeInfo.instanceBlock.color 
    : objectTypeInfo?.referenceBlock?.color 
    ? objectTypeInfo?.referenceBlock.color 
    : '#333333'

  if (!item) {
    return null
  }
  return (
    <Layer full="vertical" onEsc={clearFocus} position="right" modal={false}>
      <Box fill style={{ minWidth: '378px' }} background='#444444' border={{ side: 'left', color: objectColor, size:'medium' }}>
        <Box
          direction="row"
          align="center"
          as="header"
          justify="between"
          background='#202020'
          border={{ side: 'bottom', color: '#333333' }}
        >
          <Text margin={{ left: 'small' }} size="xlarge" style={{ textTransform: 'capitalize',color:objectColor }}>
            {objectTypeInfo.name} Information
          </Text>
          <Button icon={<FiX />} onClick={clearFocus} />
        </Box>
        <Box flex overflow="auto" pad="xsmall" border={{ color: 'black', size: 'xxsmall' }}>
          <TextInput
            placeholder="type here"
            value={item.name}
            disabled={item.canEdit}
          // onChange={e => setItemProperty(focusItem.type, focusItem.uuid, 'name', e.target.value)}
          />
          <br />
          
          <Box round="xsmall" pad="small" background="#303030" >
            <b style={{ color: 'rgba(255, 255, 255, 0.85)', paddingBottom: '2%' }} >Description : </b>
            <div>
              <TextArea
                value={item.properties.description}
                disabled={!item.canEdit}
              />
            </div>
          </Box>
          <br />

          {item.type === 'machineType' && (
            <MachineProcessList machineId={item.id}/>
          )}
            

        </Box>
        <Box
          as="footer"
          border={{ side: 'top', color: '#333333' }}
          pad="small"
          justify="end"
          direction="row"
          align="center"
        >
          <div style={{ marginInline: "30%", display: 'flex' }}>
            {item.canDelete ? (
              <div style={{ display: 'flex' }}>
                <DropButton secondary icon={<FiTrash />}
                  dropAlign={{ bottom: 'top', right: 'right' }}
                  dropProps={{ elevation: 'none' }}
                  dropContent={
                    <Box
                      background="grey"
                      pad="small"
                      round="xxsmall"
                      border={{ color: 'white', size: 'xsmall' }}
                      align="center"
                      elevation="none"
                      justify="center"

                    >
                      <Text>
                        Are you sure you want to delete this item?
                      </Text>
                      <div style={{ paddingTop: "5%" }}>
                        <Button primary icon={<FiTrash />} label="Delete" color="#ab4646" />

                      </div>

                    </Box>
                  }

                  disabled={!item.canDelete}

                  label="Delete" color="#ab4646"
                />
              </div>
            ) : (
              <Button secondary icon={<FiTrash />} disabled={!item.canDelete} label="Delete" color="#ab4646" />
            )}
          </div>
        </Box>
      </Box>
    </Layer>
  )



  // function handleProcessClick(id) {
  //   //console.log(id);
  //   for (const [key, value] of Object.entries(data)) {
  //     if (value.type === "processType" && value.id === id) {
  //       //console.log('entered1');
  //       setProcessItem(value);
  //     }
  //   }
  // }

  // function handleInputOutputClick(id, position, rotation) {

  //   for (const [key, value] of Object.entries(data)) {
  //     if (value.id === id) {
  //       setInputOutputItem(value);
  //       setPosition(position);
  //       setRotation(rotation);
  //     }
  //   }

  // }

  // if (inputOutputItem !== null) {
  //   // console.log("InputOutputDetial", item);
  //   return (
  //     <>
  //       <InputOutputDetail item={inputOutputItem} position={position} rotation={rotation} />
  //     </>
  //   )

  // }
  // else if (processItem !== null) {
  //   return (
  //     <>
  //       {processItem.type === 'processType' && (
  //         <><ProcessDetail item={processItem}
  //           inputOutputClick={(id, position, rotation) => handleInputOutputClick(id, position, rotation)} /></>
  //       )}
  //     </>
  //   )
  // } else if (focusItem.uuid && !EDITOR_TYPES.includes(focusItem.type)) {
  //   return (
  //     <>
  //       {item.type === 'locationType' && (
  //         <LocationDetail uuid={item.id} />
  //       )}
  //       {item.type === 'machineType' && (
  //         <>
  //           <div>
  //             <MachineDetail item={item} objectTypeInfo={objectTypeInfo} processClick={(id) => handleProcessClick(id)} />
  //           </div>
  //         </>

  //       )}
  //       {item.type === 'waypointType' && (
  //         <WaypointDetail uuid={item.id} />
  //       )}
  //       {item.type === 'thingTypeType' && (
  //         <ThingDetail uuid={item.id} />
  //       )}


  //     </>
  //   );
  // } else {
  //   return null
  // }


}

