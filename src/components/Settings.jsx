import React, { useRef } from 'react';
import { Layer, Box, Card, CardBody, Button, CardHeader, Notification, TextInput, List } from 'grommet';
import { FiRotateCw, FiDownload, FiUpload } from 'react-icons/fi';
import useStore from '../stores/Store';
import { saveAs } from 'file-saver';
import YAML from 'yaml';


export const SettingsModal = () => {

    const closeModal = useStore(store => store.closeModal);
    const activeModal = useStore(store => store.activeModal);
    const url = useStore(store => store.url);
    const setUrl = useStore(store => store.setUrl);
    const connection = useStore(store => store.connection);
    const connect = useStore(store => store.connect);
    const issueSettings = useStore(store => store.issueSettings);
    const updateIssueSetting = useStore(store => store.updateIssueSetting);
    const setData = useStore(store => store.setData);
    const data = useStore(store => store.programData);

    const fileInputRef = useRef();

    const download = () => {
        const text = JSON.stringify(data);

        let name = 'default_program';
        Object.values(data).some(block=>{
            if (block.type === 'programType') {
                name = block.name;
                return true
            } else {
                return false
            }
        })

        const blob = new Blob([text], { type: 'text/plain;charset=utf-8' });
        saveAs(blob, `${name.replace(' ', '_')}.json`);
    }

    const upload = async (event) => {
        const fileUploaded = event.target.files[0];
        if (fileUploaded) {
            const reader = new FileReader();
            reader.onabort = () => { message.error('Upload Aborted') }
            reader.onerror = () => { message.error('Upload Error') }
            reader.onload = () => {

                let data = YAML.parse(reader.result);
                if (data) {
                    // Do handling
                    setData(data)
                }
            }
            reader.readAsText(fileUploaded)
        }
    }

    const handleUploadClick = (_) => {
        fileInputRef.current.click();
    };

    const updateIssue = (value, issue) => {
        const item = { uuid: issue.uuid, name: issue.name, value: value };
        updateIssueSetting(item);
    }

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
                    width='large'
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

                        {/* ROS Connection (Not used currently) */}
                        {connection === 'connected' && (
                            <Notification status="normal" showIcon title="Connected!" message='You are connected to a ROS Server' round='xsmall'/>
                        )}
                        {connection === 'connecting' && (
                            <Notification status="unknown" showIcon title="Connecting..." message='You are connecting to a ROS Server' round='xsmall'/>
                        )}
                        {connection === 'disconnected' && (
                            <Notification status="warning" showIcon title="Disconnected" message='You are not connected to a ROS Server' round='xsmall'/>
                        )}

                        <Box direction='row' gap='xsmall' align='center' alignContent='center' justify='center' margin={{ top: 'small' }}>
                            <TextInput
                                placeholder="e.g. ws://localhost:9090"
                                value={url}
                                onChange={(e) => setUrl(e.target.value)}
                                size="large"
                            />
                            <Button primary size='small' icon={<FiRotateCw style={{ height: 14, width: 14 }} />} onClick={connect} />
                        </Box>

                        {/* Upload/Download */}
                        <Box direction='row' justify='around' margin={{ top: 'small' }} pad='small' background='#252525' round='xsmall'>
                            <input
                                type='file'
                                ref={fileInputRef}
                                onChange={upload}
                                style={{ display: 'none' }}
                            />
                            <Button secondary icon={<FiUpload/>} style={{flex:1,marginRight:5}} label='Upload' onClick={handleUploadClick}/>
                            <Button secondary icon={<FiDownload/>} style={{flex:1,marginLeft:5}} label='Download' onClick={download}/>
                        </Box>

                        {/* Expert Settings */}
                        <Box height='40vh' background='#252525' style={{ overflowY: 'scroll' }} margin={{ top: 'small' }} round='xsmall'>
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