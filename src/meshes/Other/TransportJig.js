import { useGLTF } from '@react-three/drei'
import TransportJigFile from './TransportJig.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(TransportJigFile);
  return [
      { type: 'group', rotation:[-Math.PI/2,0,0], scale:[1,1,1], children: [
        {
            type:'raw',
            geometry:nodes.TransportJig.geometry,
            material:materials['TransportJig.material']
          }
      ]}
    ]
}

useGLTF.preload(TransportJigFile)