import React from 'react';
import { Row } from 'antd';
import { DeleteFilled } from '@ant-design/icons';
import { useDrop } from 'react-dnd';
import { acceptLookup } from './acceptLookup';
// import useStore from '../../stores/Store';

export const DeleteZone = (_) => {

    const acceptTypes = acceptLookup.trash.all.accepts;

    // const [deleteChildPrimitive,deleteHierarchical] = useStore(state=>[state.deleteChildPrimitive,state.deleteHierarchical]);

    const [{isOver, canDrop}, drop] = useDrop({
        accept: acceptTypes,
        drop: (item, _) => {
            console.log(item)
            if (item.onDelete && item.deleteable) {
                item.onDelete(item)
            }
        },
        canDrop: (item, _) => (acceptTypes.indexOf(item.type)>=0 && item.deleteable),
        collect: (monitor) => ({
            isOver: monitor.isOver(),
            canDrop: monitor.canDrop()
        })
    })

    if (!canDrop) {
        console.log()
    }

    let opacity = 0.7;
    let width = 70;
    if (isOver) {
        width = 300;
        opacity = 0.5;
    }

    const styles = {
        backgroundColor: '#a61d24',
        width,
        zIndex: 100,
        position: 'absolute',
        right:30,
        bottom:30,
        height:70,
        borderRadius:3,
        padding:2,
        display:'flex',
        flexDirection: 'row'
    }

    return (
        <div hidden={!canDrop} ref={drop} style={styles} >
            <Row align='middle' justify='center' style={{flex:1,height:'100%',width:'100%',backgroundColor:`rgba(0,0,0,${opacity})`}}>
                <DeleteFilled style={{color:'#a61d24',fontSize:40}}/>
            </Row>
        </div>
        
    )
};

