import { useGLTF } from '@react-three/drei'
import BladeFile from './Blade.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(BladeFile);
  return [
      { type: 'group', rotation:[Math.PI,0,0], scale:[1,1,1], children: [
        {
            type:'raw',
            geometry:nodes.Blade.geometry,
            material:materials['Blade.material'],
            
          }
      ]}
    ]
}

useGLTF.preload(BladeFile)