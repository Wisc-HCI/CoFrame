import React, {forwardRef} from 'react';

export const PrimitiveBlock = forwardRef((props, ref) => {

    // TODO: Render differently depending on the primitive properties

    const styles = {
        backgroundColor: props.data.type === 'node.primitive.skill-call' ? '#62869e' : '#629e6c',
        minHeight:30,
        borderRadius:3,
        margin:4,
        padding:5
    }

    return (
        <div {...props} ref={ref} style={{...props.style, ...styles}}>
            {props.data.name}
        </div>
    )
});

