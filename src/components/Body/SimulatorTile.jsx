import React, {forwardRef} from "react";
import { Box } from "grommet";
import { Scene } from "robot-scene";
import useStore from "../../stores/Store";
import { Controls } from "../Elements/Controls";
import { InfoTile } from "./InfoTile";
import useMeasure from "react-use-measure";
import { ReflexContainer, ReflexSplitter, ReflexElement } from "react-reflex";
// import { useSpring, animated } from '@react-spring/web';
// import { config } from 'react-spring';
import Tile from "../Elements/Tile";

import MeshLookupTable from "../../meshes";
import shallow from "zustand/shallow";

export const SimulatorTile = forwardRef((_,ref) => {
  const primaryColor = useStore((state) => state.primaryColor,shallow);
  const clearFocus = useStore((state) => state.clearFocus,shallow);
  const tfVisible = useStore((state) => state.tfVisible,shallow);
  const paused = useStore((state) => state.focus.length === 0,shallow);
  const [topRef, topBounds] = useMeasure();
  const [bottomRef, bottomBounds] = useMeasure();
  // const containerStyle = useSpring({ flex: visible ? 11 : 0, config: config.stiff });
//   console.log({ topBounds, bottomBounds });

  return (
    <Box
      ref={ref}
      animation="fadeIn"
      flex
      direction="column"
      width="100%"
      height="100%"
      gap="xsmall"
      background='rgb(25,25,25)'
    >
      <ReflexContainer orientation="horizontal">
        <ReflexElement style={{ overflow: "hidden" }} minSize={200}>
          <Box ref={topRef} fill>
            <Tile
              style={{ height: topBounds.height, paddingBottom: "4pt",overflow:'hidden'}}
              borderRadius={0}
              backgroundColor={primaryColor}
              borderWidth={3}
              internalPaddingWidth={2}
              header={
                <Box
                  direction="row"
                  justify="between"
                  align="center"
                  pad={{ right: "small" }}
                >
                  <h3 style={{ margin: "10pt" }}>Simulator</h3>
                  <Controls />
                </Box>
              }
            >
              <div
                style={{
                  height: topBounds.height - 65,
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
          </Box>
        </ReflexElement>
        <ReflexSplitter />
        <ReflexElement minSize={200}>
          <Box fill ref={bottomRef}>
            <InfoTile maxHeight={bottomBounds.height-8} />
          </Box>
        </ReflexElement>
      </ReflexContainer>
    </Box>
  );
});
