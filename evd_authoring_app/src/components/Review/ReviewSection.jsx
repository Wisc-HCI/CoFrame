import React from 'react';

import { List, Switch, Collapse, Space, Badge } from 'antd';
import { CaretRightOutlined } from '@ant-design/icons';

import './custom.css'

export function ReviewSection(props) {

  const { title, complete, enabled, items } = props;

  return (
    <List.Item style={{margin:0,padding:0}}>
        <Collapse
            collapsible={enabled ? null : 'disabled'} 
            expandIcon={({ isActive }) => <CaretRightOutlined style={{color:'white'}} rotate={isActive ? 90 : 0} />}
            style={{
                flex:1, 
                borderRadius: 3,
                opacity: enabled ? 1 : 0.5,
                backgroundColor: '#1f1f1f',
                marginTop: 5, marginLeft: 5, marginRight: 5, marginBottom:0}}
            expandIconPosition='right'
        >
            <Collapse.Panel style={{margin:0}} header={<span style={{color:'white'}}>{title}</span>} key="1" extra={<Switch checkedChildren='Done!' checked={complete}/>}>
                <List
                    split={false}
                    dataSource={items}
                    renderItem={(item)=>(
                        <List.Item 
                            extra={
                                <Space>
                                    {item.force && !item.complete && (
                                        <Badge danger count={'Change Required'}/>
                                    )}
                                    <Switch size='small' disabled={item.force} defaultChecked={item.complete}/>
                                </Space>
                            }
                            style={{
                                borderRadius: 3,
                                backgroundColor: '#1f1f1f',
                                margin: 5, padding: 10
                            }}
                        >
                            <List.Item.Meta title={item.label} />
                        </List.Item>
                    )}
                />
            </Collapse.Panel>
        </Collapse>
    </List.Item>
  );
};