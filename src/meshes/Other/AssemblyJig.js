import { useGLTF } from '@react-three/drei'
import AssemblyJigFile from './AssemblyJig.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(AssemblyJigFile);
  return [
      { type: 'group', rotation:[-Math.PI/2,0,0], scale:[1,1,1], children: [
        {
            type:'raw',
            geometry:nodes.AssemblyJig.geometry,
            material:materials.AssemblyJigMaterial,
            
          }
      ]}
    ]
}

useGLTF.preload(AssemblyJigFile)