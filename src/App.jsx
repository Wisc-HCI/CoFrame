import React from "react";
// import { FiSettings } from "react-icons/fi";
import { ReviewTile } from "./components/Body/ReviewTile";
import { SimulatorTile } from "./components/Body/SimulatorTile";
import { ProgramTile } from "./components/Body/ProgramTile";
import { Grommet } from "grommet";
import ParentSize from "@visx/responsive/lib/components/ParentSize";
import { TIMELINE_TYPES } from "./stores/Constants";
// import { Modals } from "./components/Modals";
import { Detail } from "./components/Detail";
import { SettingsModal } from "./components/Settings";
import TimelineGraph from "./components/TimelineGraph";
import { ThemeProvider, createTheme } from "@mui/material/styles";
// import useMeasure from 'react-use-measure';

// import { CoFrameIcon } from "./components/Icon";

import useStore from "./stores/Store";

// import { useSpring, animated } from "@react-spring/web";
import { motion } from "framer-motion";
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
  // const mainStyle = useSpring({ height: visibleSteps ? "80vh" : "100vh" });
  // const drawerStyle = useSpring({ height: visibleSteps ? "20vh" : "0vh" });
  // const leftStyle = useSpring({
  //   flex: viewMode === "default" || viewMode === "sim" ? 45 : 0,
  // });
  // const rightStyle = useSpring({
  //   flex: viewMode === "default" || viewMode === "program" ? 55 : 0,
  // });

  const mainVariants = {
    closedDrawer: {height: "100vh"},
    openDrawer: {height: "80vh"}
  }

  const drawerVariants = {
    closedDrawer: {height: "0vh"},
    openDrawer: {height: "20vh"}
  }

  const leftVariants = {
    visible: {flex:45},
    hidden: {flex:0}
  }

  const rightVariants = {
    visible: {flex:55},
    hidden: {flex:0}
  }

  const theme = getTheme(primaryColor);
  const muiTheme = createTheme({
    palette: {
      mode: "dark",
      primaryColor: {
        main: primaryColor,
      },
      quiet: {
        main: "#444",
        darker: "#333",
      },
    },
  });

  return (
    <Grommet full theme={theme}>
      {/* Main container */}
      <ThemeProvider theme={muiTheme}>
        <div
          style={{
            backgroundColor: "black",
            height: "100vh",
            width: "100vw",
            position: "fixed",
          }}
        >
          <motion.div
            layout
            variants={mainVariants}
            animate={visibleSteps ? 'openDrawer' : 'closedDrawer'}
            style={{
              flexDirection: "row",
              display: "flex",
            }}
          >
            <div style={{ width: 350, height: "100%" }}>
              <ReviewTile />
            </div>

            <motion.div
            layout
            variants={leftVariants}
            animate={viewMode === "default" || viewMode === "sim" ? 'visible' : 'hidden'}
              style={{
                overflow: "hidden",
              }}
            >
              <SimulatorTile visible />
            </motion.div>
            <motion.div
            layout
            variants={rightVariants}
            animate={viewMode === "default" || viewMode === "program" ? 'visible' : 'hidden'}
              style={{
                overflow: "hidden",
              }}
            >
              <ProgramTile visible />
            </motion.div>
          </motion.div>
          <motion.div
          layout
            variants={drawerVariants}
            animate={visibleSteps ? 'openDrawer' : 'closedDrawer'}
            style={{
              backgroundColor: "#444444",
              borderTop: `5px solid ${primaryColor}`,
            }}
          >
            <ParentSize>
              {({ width, height }) => (
                <TimelineGraph
                  width={width}
                  height={height - 10}
                  visible={visibleSteps}
                />
              )}
            </ParentSize>
          </motion.div>
        </div>
        {/* <Modals /> */}
        <SettingsModal />
        <Detail />
      </ThemeProvider>
    </Grommet>
  );
}
