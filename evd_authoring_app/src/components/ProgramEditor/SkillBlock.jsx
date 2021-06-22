import React, {useCallback} from 'react';
import { useDrag } from 'react-dnd';
import useEvdStore from '../../stores/EvdStore';

const style = {
    position: 'absolute',
    border: '1px solid gray',
    borderRadius: 3,
    backgroundColor: 'white',
    padding: '0.5rem 1rem',
    cursor: 'move',
};

export const SkillBlock = ({ uuid }) => {
    
    const {item,setItemProperty} = useEvdStore(useCallback(state=>({
        item:state.data.primitives[uuid],
        setItemProperty:state.setItemProperty
    }),[uuid]))
    
    const [{ isDragging }, drag] = useDrag(() => ({
        type: 'function',
        item: { id:uuid, left:item.left, top:item.top },
        collect: (monitor) => ({
            isDragging: monitor.isDragging(),
        }),
    }), [uuid, item]);
    
    if (isDragging) {
        return <div ref={drag} />;
    }

    return (
        <div ref={drag} style={{ ...style, left:item.left, top:item.top }} role="Box">
            {item.name}
        </div>
    );
};
