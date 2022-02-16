import React from 'react';
import { Layer, Box, Card, CardBody, Button, CardHeader, Notification, TextInput, List } from 'grommet';
import { FiRotateCw } from 'react-icons/fi';
import useStore from '../stores/Store';


export const SettingsModal = () => {

    const closeModal = useStore(store => store.closeModal);
    const activeModal = useStore(store => store.activeModal);
    const url = useStore(store => store.url);
    const setUrl = useStore(store => store.setUrl);
    const connection = useStore(store => store.connection);
    const connect = useStore(store => store.connect);
    const issueSettings = useStore(store => store.issueSettings);
    const updateIssueSetting = useStore(store => store.updateIssueSetting);

    const connectButtonText = connection === 'connecting'
        ? 'Connecting'
        : connection === 'connected'
            ? 'Reconnect'
            : 'Connect';

    const updateIssue = (value, issue) => {
        const item = { uuid: issue.uuid, name: issue.name, value: value };
        updateIssueSetting(item);
    }

    console.log(activeModal)
    if (activeModal) {
        return (
            <Layer
                id='settings modal'
                position='center'
                onClickOutside={closeModal}
                onEsc={closeModal}
            >
                <Card
                    round='xsmall'
                    background='#555555'
                    width='medium'
                    elevation='none'
                >
                    <CardHeader
                        background='#333333'
                        pad='small'
                        border={{ side: 'bottom', color: '#333333' }}
                    >
                        Settings
                    </CardHeader>
                    <CardBody pad='small'>
                        {connection === 'connected' && (
                            <Notification status="normal" showIcon title="Connected!" message='You are connected to a ROS Server'/>
                        )}
                        {connection === 'connecting' && (
                            <Notification status="unknown" showIcon title="Connecting..." message='You are connecting to a ROS Server'/>
                        )}
                        {connection === 'disconnected' && (
                            <Notification status="warning" showIcon title="Disconnected" message='You are not connected to a ROS Server'/>
                        )}

                        <Box direction='row' gap='xsmall' align='center' alignContent='center' justify='center' margin={{top:'small'}}>
                            <TextInput
                                placeholder="e.g. ws://localhost:9090"
                                value={url}
                                onChange={(e) => setUrl(e.target.value)}
                                size="large"
                            />
                            <Button primary size='small' icon={<FiRotateCw style={{height:14,width:14}}/>} onClick={connect} />
                        </Box>
                        
                        <Box height='40vh' background='#252525' style={{ overflowY: 'scroll' }} margin={{top:'small'}}>
                            <List data={Object.values(issueSettings)} style={{ padding: 5 }} margin='none' pad='none'>
                                {(entry, idx) => (
                                    <Box animation={{ type: 'fadeIn', delay: idx * 100 }} direction='row' key={entry.name.concat('div')} margin='small' pad='xsmall'>
                                        {entry.name}
                                        {!entry.max && <TextInput
                                            type='number'
                                            key={entry.name.concat('input')}
                                            min={entry.min}
                                            defaultValue={entry.value}
                                            onChange={(e) => updateIssue(e, entry)} />
                                        }
                                        {entry.max && <TextInput
                                            type='number'
                                            key={entry.name.concat('input')}
                                            min={entry.min}
                                            max={entry.max}
                                            defaultValue={entry.value}
                                            onChange={(e) => updateIssue(e, entry)} />
                                        }
                                    </Box>
                                )}
                            </List>
                        </Box>
                        {issueSettings && Object.values(issueSettings).map((entry) => {
                            return
                        })}
                    </CardBody>
                    {/* <CardFooter>footer</CardFooter> */}
                </Card>
            </Layer>
        )
    } else return null;
}