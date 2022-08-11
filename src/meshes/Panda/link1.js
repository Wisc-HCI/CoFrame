/*
Auto-generated by: https://github.com/pmndrs/gltfjsx
*/

import { useGLTF } from "@react-three/drei";
import Link1 from "./link1.glb";

export default function Model(props) {
  const { nodes, materials } = useGLTF(Link1);
  return [
    {
      type: "group",
      position: [0, -0.192, 0],
      rotation: [-Math.PI / 2, 0, 0],
      children: [
        {
          type: "raw",
          geometry: nodes.node0.geometry,
          material: materials.Part__Feature_001,
        },
      ],
    },
  ];
}

useGLTF.preload(Link1);
