import React from "react";
// import { FiSettings } from "react-icons/fi";
import { ReviewTile } from "./components/Body/ReviewTile";
import { SimulatorTile } from "./components/Body/SimulatorTile";
import { ProgramTile } from "./components/Body/ProgramTile";
import { Grommet, Box } from "grommet";
import ParentSize from "@visx/responsive/lib/components/ParentSize";
import { TIMELINE_TYPES } from "./stores/Constants";
// import { Modals } from "./components/Modals";
import { Detail } from "./components/Detail";
import { SettingsModal } from "./components/Settings";
import Graph from "./components/Graph";
// import useMeasure from 'react-use-measure';

// import { CoFrameIcon } from "./components/Icon";

import useStore from "./stores/Store";

import { useSpring, animated } from "@react-spring/web";
import { getTheme } from "./theme";

export default function App() {
  const primaryColor = useStore((state) => state.primaryColor);
  const viewMode = useStore((state) => state.viewMode);
  const visibleSteps = useStore((state) =>
    state.focus.some((focusItem) =>
      TIMELINE_TYPES.includes(state.programData[focusItem]?.type)
    )
  );
  //   console.warn("visibleSteps", visibleSteps);
  //   const bodyStyle = useSpring({ height: visibleSteps ? "80vh" : "100vh" });
  const mainStyle = useSpring({ height: visibleSteps ? "80vh" : "100vh" });
  const drawerStyle = useSpring({ height: visibleSteps ? "20vh" : "0vh" });
  const leftStyle = useSpring({
    flex: viewMode === "default" || viewMode === "sim" ? 45 : 0,
  });
  const rightStyle = useSpring({
    flex: viewMode === "default" || viewMode === "program" ? 55 : 0,
  });

  const theme = getTheme(primaryColor);

  return (
    <Grommet full theme={theme}>
      {/* Main container */}
      <div
        style={{
          backgroundColor: "black",
          height: "100vh",
          width: "100vw",
          position: "fixed",
        }}
      >
        <animated.div
          style={{
            ...mainStyle,
            flexDirection: "row",
            display: "flex",
          }}
        >
          <div
            style={{ width: 350, height: "100%"}}
          >
            <ReviewTile />
          </div>

          <animated.div
            style={{
              ...leftStyle,
              overflow: "hidden",
            }}
          >
            <SimulatorTile
              visible
            />
          </animated.div>
          <animated.div
            style={{
              ...rightStyle,
              overflow: "hidden",
            }}
          >
            <ProgramTile
              visible
            />
          </animated.div>
        </animated.div>
        <animated.div style={{ ...drawerStyle, backgroundColor:"#444444", borderTop: `5px solid ${primaryColor}` }}>
          <ParentSize>
            {({ width, height }) => (
              <Graph width={width} height={height - 10} />
            )}
          </ParentSize>
        </animated.div>
      </div>
      <SettingsModal />
      <Detail />
      {/* <Modals /> */}
    </Grommet>
  );
}
