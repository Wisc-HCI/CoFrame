import {useDroppable} from '@dnd-kit/core';
import {Grid} from './Grid'

export const Canvas = (props) => {
  // Do your draggable stuff here
  const {isOver, setNodeRef} = useDroppable({
   id:props.id,
 });
  return (
    <div ref={setNodeRef}>
    <Grid >
    {props.children}
    </Grid>
    </div>
  )
}
