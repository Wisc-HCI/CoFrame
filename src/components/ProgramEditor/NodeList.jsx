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
        paddingLeft: 5,
        paddingRight: 5,
        textAlign: 'center',
        fontSize: 14
    }

    return (
        <div className='nodrag' style={containerStyle}>

            {uuids.length === 0 ? (
                <DropRegion key={`drop-${field}`} parentId={ancestors[0].uuid} field={field} idx={0} ancestors={ancestors} height={44} text={'No Items'} dropDisabled={dropDisabled}/>
            ) : (
                <DropRegion key={`drop-${field}`} parentId={ancestors[0].uuid} field={field} idx={0} ancestors={ancestors} height={6} dropDisabled={dropDisabled}/>
            )}
            {uuids.map((uuid,idx)=>(
                <React.Fragment key={uuid}>
                    <Block key={`block-${uuid}`} field={field} idx={idx} ancestors={ancestors} uuid={uuid} context={context} dragDisabled={dragDisabled} dropDisabled={dropDisabled} fromNodeList/>
                    <DropRegion key={`drop-${uuid}`} parentId={ancestors[0].uuid} field={field} idx={idx+1} ancestors={ancestors} height={6} dropDisabled={dropDisabled}/>
                </React.Fragment>
            ))}
        </div>
    )
};