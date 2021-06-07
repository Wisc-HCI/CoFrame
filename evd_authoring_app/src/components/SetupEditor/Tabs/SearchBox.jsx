import React, { useState } from 'react';

import { Tabs, Card, Button, Input, Space } from 'antd';
import { PlusOutlined, SearchOutlined, CloseOutlined } from '@ant-design/icons';
import useGuiStore from '../../../stores/GuiStore';

export function SearchBox(props) {
    const { searchTerm, changeVisibility, clearSearch, visible, onChange, buttonVisible } = props;

    return (
        <Space>
            <Input 
                allowClear
                value={searchTerm} 
                placeholder="Search..." 
                onChange={onChange} 
                addonAfter={<CloseOutlined onClick={changeVisibility, clearSearch}/>}
                style={{ maxWidth: 300, minWidth: 100, display: visible ? "block" : "none" }} />


            <Button style={{ left: "-5px", display: buttonVisible ? "block" : "none" }} type='outline' icon={<SearchOutlined />} onClick={changeVisibility} />
            <Button
                type='outline'
                icon={<PlusOutlined />}
            />

        </Space>

    );


};
