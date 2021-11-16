import React from "react";
import { Block } from "./Blocks";
import { DropRegion } from "./DropRegion";
// import {SortableContext} from '@dnd-kit/sortable';

export const NodeList = ({ ancestors, field, uuids, context, dragDisabled, dropDisabled }) => {

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
        <div className='nodrag' style={containerStyle}>
            {uuids.length === 0 ? (
                <DropRegion parentId={ancestors[0].uuid} field={field} idx={0} ancestors={ancestors} height={44} text={'No Items'} dropDisabled={dropDisabled}/>
            ) : uuids.map((uuid,idx)=>(
                <React.Fragment>
                    <Block key={uuid} ancestors={ancestors} uuid={uuid} context={context} dragDisabled={dragDisabled}/>
                    <DropRegion parentId={ancestors[0].uuid} field={field} idx={idx} ancestors={ancestors} height={6} dropDisabled={dropDisabled}/>
                </React.Fragment>
            ))}
        </div>
    )
};