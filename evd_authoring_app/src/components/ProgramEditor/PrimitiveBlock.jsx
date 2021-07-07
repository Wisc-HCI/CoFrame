import React, {forwardRef} from 'react';
import { ParameterZone } from './ParameterZone';
import useEvdStore from '../../stores/EvdStore';


export const PrimitiveBlock = forwardRef((props, ref) => {

    // TODO: Render differently depending on the primitive properties
    const setPrimitiveParameter = useEvdStore(state => state.setPrimitiveParameter);

    const styles = {
        backgroundColor: props.data.type === 'node.primitive.skill-call.' ? '#62869e' : '#629e6c',
        minHeight:30,
        minWidth:200,
        borderRadius:3,
        margin:4,
        padding:5
    }

    if (props.data.parameters.machine_uuid) {
        console.log('found one')
        console.log(props.data.parameters.machine_uuid)
    }

    return (
        <div {...props} ref={ref} style={{...props.style, ...styles}}>
            <div style={{fontSize:16}}>{props.data.name}</div>
            {props.data.type.includes('node.primitive.machine-primitive') && (
                <div>Machine: <ParameterZone
                                displayText={props.data.parameters.machine_uuid}
                                acceptTypes={['node.machine.']}
                                itemType='machine'
                                canRemove={props.data.editable}
                                onRemove={()=>console.log('delete param')}
                                onDrop={(data)=>console.log(data)}/>
                </div>
            )}
        </div>
    )
});
