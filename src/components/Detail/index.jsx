import { React } from 'react';
import { MachineProcessList } from './MachineDetail';
import { ProcessIOList } from './ProcessDetail';
import PositionRotationTF from './PositionRotationTF';
import { TextArea, Text, Box, TextInput, Button, Layer, DropButton, Spinner } from 'grommet';
import { FixtureItem } from './FixtureDetail'

import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';
import { FiTrash, FiX } from 'react-icons/fi';
import { NumberInput } from '../NumberInput';
import { DETAIL_TYPES, STATUS } from '../../stores/Constants';
import JointGripperInput from "./JointGripperInput";
import LocationWaypointDetail from './LocationWaypointDetail';
export const Detail = (_) => {


  const {
    item,
    objectTypeInfo
  } = useStore(state => {
    let item = null;

    state.focus.slice().reverse().some(v => {

      if (state.programData[v] && state.activeFocus === v && DETAIL_TYPES.includes(state.programData[v].type)) {
        item = state.programData[v];

        return true
      } else {

        return false
      }
    })

    // console.log("state:", state.activeFocus);
    return {
      item,
      objectTypeInfo: item?.type ? state.programSpec.objectTypes[state.programData[item?.id].type] : null
    }
  }, shallow);
  console.log("objectTypeInfo", objectTypeInfo);

  //const addFocusItem = useStore(state => state.addFocusItem);
  const clearFocus = useStore(state => state.clearFocus);
  const updateItemName = useStore(state => state.updateItemName);
  const updateItemSimpleProperty = useStore(state => state.updateItemSimpleProperty);
  const updateItemDescription = useStore(state => state.updateItemDescription);

  const objectColor = objectTypeInfo?.instanceBlock?.color
    ? objectTypeInfo.instanceBlock.color
    : objectTypeInfo?.referenceBlock?.color
      ? objectTypeInfo?.referenceBlock.color
      : '#333333'


  const deleteBlock = useStore(state => state.deleteBlock);
  console.log("item:", item);










  if (!item) {
    return null
  }

  return (
    <Layer full="vertical" onEsc={clearFocus} position="right" modal={false}>
      <Box fill style={{ minWidth: '378px' }} background='#444444' border={{ side: 'left', color: objectColor, size: 'medium' }}>
        <Box
          direction="row"
          align="center"
          as="header"
          justify="between"
          background='#202020'
          border={{ side: 'bottom', color: '#333333' }}
        >
          <Text margin={{ left: 'small' }} size="xlarge" style={{ textTransform: 'capitalize', color: objectColor }}>
            {objectTypeInfo.name} Information
          </Text>
          {item.properties.status === STATUS.PENDING && <Spinner/>}
          <Button icon={<FiX />} onClick={clearFocus} />
        </Box>
        <Box flex overflow="auto" pad="xsmall" gap='small' border={{ color: 'black', size: 'xxsmall' }}>
          <TextInput
            placeholder="type here"
            value={item.name}
            disabled={!item.canEdit}
            onChange={e => updateItemName(item.id, e.target.value)}
          />

          <Box round="xsmall" pad="small" background="#303030" wrap={true} >
            <b style={{ color: 'rgba(255, 255, 255, 0.85)', paddingBottom: '2%' }} >Description : </b>
            <Box>
              <TextArea
                value={item.properties.description}
                disabled={!item.canEdit}
                resize='vertical'
                onChange={e => updateItemDescription(item.id, e.target.value)}
              />
            </Box>
          </Box>

          {item.properties.processTime !== undefined && (
            <Box direction='row' background='#303030' round="xsmall" pad="small" style={{ marginBottom: 5 }} justify='between' wrap={true}>
              <b style={{ color: 'rgba(255, 255, 255, 0.85)' }} >Time : </b>
              <Box direction='row' >
                <NumberInput
                  value={item.properties.processTime}
                  min={0}
                  max={Infinity}
                  onChange={(value) => updateItemSimpleProperty(item.id, 'processTime', value)}
                  disabled={!item.canDelete}
                  visualScaling={1 / 1000} />
                <b style={{ color: 'rgba(255, 255, 255, 0.85)', paddingLeft: "4%" }} >sec</b>
              </Box>
            </Box>
          )}


          {item.properties.position !== undefined && item.properties.rotation !== undefined && (
            <>
              <Box wrap={true}>
                <PositionRotationTF itemID={item.id} position={item.properties.position}
                  rotation={item.properties.rotation} />
              </Box>
            </>
          )}

          {item.type === 'machineType' && (
            <>
              <MachineProcessList machineId={item.id} />
              <br />
            </>

          )}

          {item.type === 'processType' && (
            <>
              <ProcessIOList processId={item.id} isInput />
              <ProcessIOList processId={item.id} />
            </>

          )}
          {item.properties.relativeTo !== undefined && item.properties.relativeTo !== "world" &&
            item.properties.relativeTo !== null && (

              <>
                <Box>
                  <FixtureItem fixtureID={item.properties.relativeTo} />
                </Box>

              </>
            )}

          {item.type === 'gripperType' && (
            <>
              <JointGripperInput robotID={item.id} isGripper={true} />
            </>
          )

          }
          {item.type === 'robotAgentType' && (
            <>
              <JointGripperInput robotID={item.id} isGripper={false} />
            </>
          )}
          {(item.type === 'locationType' || item.type === 'waypointType') && (
            <>
              <LocationWaypointDetail itemID={item.id} />
            </>
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
                        <Button primary icon={<FiTrash />} label="Delete" color="#ab4646" onClick={() =>
                          deleteBlock(item, "spawner", objectTypeInfo)
                        } />

                      </div>

                    </Box>
                  }

                  disabled={!item.canDelete}

                  label="Delete" color="#ab4646"
                />
              </div>
            ) : (
              <Button secondary icon={<FiTrash />} disabled={!item.canDelete} label="Delete" color="#ab4646"

              />
            )}
          </div>
        </Box>
      </Box>
    </Layer>
  )





}

