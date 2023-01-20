import React, {forwardRef} from "react";
import { Scene } from "robot-scene";
import useStore from "../../stores/Store";
import { Controls } from "../Elements/Controls";
import { InfoTile } from "./InfoTile";
import useMeasure from "react-use-measure";
import { ReflexContainer, ReflexSplitter, ReflexElement } from "react-reflex";
// import { useSpring, animated } from '@react-spring/web';
// import { config } from 'react-spring';
import Tile from "../Elements/Tile";
import { Stack, Typography } from "@mui/material";
import MeshLookupTable from "../../meshes";
import shallow from "zustand/shallow";
import ParentSize from "@visx/responsive/lib/components/ParentSize";

export const SimulatorTile = forwardRef((_,ref) => {
  const primaryColor = useStore((state) => state.primaryColor,shallow);
  const clearFocus = useStore((state) => state.clearFocus,shallow);
  const tfVisible = useStore((state) => state.tfVisible,shallow);
  const paused = useStore((state) => state.focus.length === 0,shallow);

  return (
    <Stack
      ref={ref}
      style={{
        display:'flex',
        width:'100%',
        height:'100%',
        backgroundColor:'rgb(25,25,25)'
      }}
      spacing={0.5}
      
    >
      <ReflexContainer orientation="horizontal">
        <ReflexElement style={{ overflow: "hidden" }} minSize={200}>
          <ParentSize>
            {({ height }) => (
              <Tile
              style={{ height, paddingBottom: "4pt",overflow:'hidden'}}
              borderRadius={0}
              backgroundColor={primaryColor}
              borderWidth={3}
              internalPaddingWidth={2}
              header={
                <Stack
                  style={{
                    justifyContent:'space-between',
                    alignContent:'center'
                  }}
                  direction="row"
                  justify="between"
                  align="center"
                  pad={{ right: "small" }}
                >
                  <Typography style={{ margin: "10pt", color:'white'}}>Simulator</Typography>
                  <Controls />
                </Stack>
              }
            >
              <div
                style={{
                  height: height - 62,
                  width: "100%",
                  backgroundColor: "black",
                  padding: 0,
                }}
              >
                <Scene
                  displayTfs={tfVisible}
                  displayGrid={true}
                  isPolar={false}
                  backgroundColor="#1e1e1e"
                  planeColor="#141414"
                  highlightColor={primaryColor}
                  plane={-0.75}
                  fov={50}
                  store={useStore}
                  onPointerMissed={clearFocus}
                  paused={paused}
                  meshLookup={MeshLookupTable}
                />
              </div>
            </Tile>
            )}
          </ParentSize>
        </ReflexElement>
        <ReflexSplitter />
        <ReflexElement minSize={200}>
          <ParentSize>
            {({height})=>(
              <InfoTile maxHeight={height-8} />
            )}
          </ParentSize>
        </ReflexElement>
      </ReflexContainer>
    </Stack>
  );
});
