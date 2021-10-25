import React from "react";
import { Block } from "./Blocks";
import {SortableContext} from '@dnd-kit/sortable';

export const NodeList = ({ ancestors, uuids, context, dragDisabled }) => {

    const containerStyle = {
        backgroundColor: 'rgba(0,0,0,0.5)',
        borderRadius: 5,
        minWidth: 110,
        minHeight: 54,
        padding: 5,
        textAlign: 'center',
        fontSize: 14
    }

    return (
        <SortableContext items={uuids} id={ancestors[0].uuid}>
            <div style={containerStyle}>
                {uuids.map(uuid=>(
                    <Block ancestors={ancestors} uuid={uuid} context={context} dragDisabled={dragDisabled}/>
                ))}
            </div>
        </SortableContext>
    )
};