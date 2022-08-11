/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/

import Ur5Forearm from './forearm.glb';
import { useGLTF } from '@react-three/drei';

export default function Model(props) {

  const { nodes, materials } = useGLTF(Ur5Forearm);
  return [

{type:'raw', geometry:nodes.Actor0_001.geometry, material:materials.blau_005} ,
{type:'raw', geometry:nodes.Actor1.geometry, material:materials.Rohr_005},
{type:'raw', geometry:nodes.Actor2.geometry, material:nodes.Actor2.material},
{type:'raw', geometry:nodes.Actor3.geometry, material:nodes.Actor3.material}

]
}

useGLTF.preload(Ur5Forearm)