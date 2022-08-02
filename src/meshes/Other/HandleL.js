import { useGLTF } from '@react-three/drei'
import LHandleFile from './HandleL.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(LHandleFile);
  return [
      { type: 'group', rotation:[Math.PI,0,0], scale:[1,1,1], children: [
        {
            type:'raw',
            geometry:nodes.LHandle.geometry,
            material:materials.lmaterial
          }
      ]}
    ]
}

useGLTF.preload(LHandleFile)       