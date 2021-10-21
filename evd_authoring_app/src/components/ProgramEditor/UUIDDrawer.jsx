import React, {useCallback} from 'react';
// import { StaticSortable } from './Wrappers';
import { acceptLookup } from './acceptLookup';
import useStore from "../../stores/Store";
import shallow from 'zustand/shallow';
import { typeToKey, objectMap } from '../../stores/helpers';
import { UUIDBlock } from './UUIDBlock';
import lodash from 'lodash';

export const UUIDDrawer = ({itemType}) => {
    // itemType is the "type" of item from the EvdStore that the uuid corresponds to.
    // e.g. 'thingType', 'machine', 'waypoint', etc.
    const data = useStore(useCallback(state=>Object.values(state.data).filter(v=>v.type===itemType),[itemType]),shallow);

    const nameLookup = useStore(state => lodash.filter(state.data, v=>['thing','location','waypoint','machine','trajectory'].includes(v.type)), shallow)

    const ancestors = [
        {uuid:'drawer',...acceptLookup.drawer.default}
    ];

    return (
        <React.Fragment>
            {data.map((datum,i)=>(
                <div key={datum.uuid} style={{paddingTop:5}} >
                    <UUIDBlock 
                        ancestors={ancestors} 
                        idx={i} 
                        parentData={{type:'drawer',uuid:'drawer'}}
                        data={{...datum,itemType,type:`uuid-${itemType}`}} 
                        dragBehavior='copy'
                        dropDisabled
                        context={nameLookup}/>
                </div>
            ))}
        </React.Fragment>
    );
};
