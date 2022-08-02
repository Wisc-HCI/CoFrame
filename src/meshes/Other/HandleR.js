import { useGLTF } from '@react-three/drei'
import RHandleFile from './HandleR.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(RHandleFile);
  return [
      { type: 'group', rotation:[Math.PI,0,0], scale:[1,1,1], children: [
        {
            type:'raw',
            geometry:nodes.RHandle.geometry,
            material:materials.rmaterial,
          }
      ]}
    ]
}

useGLTF.preload(RHandleFile)      