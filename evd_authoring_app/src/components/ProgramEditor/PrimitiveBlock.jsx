import React, {forwardRef} from 'react';

export const PrimitiveBlock = forwardRef((props, ref) => {

    // TODO: Render differently depending on the primitive properties

    const styles = {
        backgroundColor: props.data.type === 'node.primitive.skill-call.' ? '#62869e' : '#629e6c',
        minHeight:30,
        minWidth:200,
        borderRadius:3,
        margin:4,
        padding:5
    }

    return (
        <div {...props} ref={ref} style={{...props.style, ...styles}}>
            <div style={{fontSize:16}}>{props.data.name}</div>
            {props.data.type.includes('node.primitive.machine-primitive') && (
                <div>Machine: <div style={{minHeight: 20,backgroundColor:'grey',borderRadius:5,padding:3,textAlign:'center',boxShadow:'inset 0pt 0pt 2pt 1pt rgba(0,0,0,0.5)'}}>{props.data.parameters.machine_uuid?props.data.parameters.machine_uuid:'No Machine'}</div></div>
            )}
        </div>
    )
});

