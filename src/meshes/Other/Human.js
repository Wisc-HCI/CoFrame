import { useGLTF } from '@react-three/drei'
import HumanFile from './Human.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(HumanFile);
  return [
      { type: 'group', rotation:[Math.PI,0,0], scale:[1,1,1], children: [
        {
            type:'raw',
            geometry:nodes.Human.geometry,
            material:materials.rmaterial,
          }
      ]}
    ]
}

useGLTF.preload(HumanFile)      