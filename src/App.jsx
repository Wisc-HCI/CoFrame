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
import TimelineGraph from "./components/TimelineGraph";
import {
  ThemeProvider,
  createTheme,
  // styled,
  // useTheme,
} from "@mui/material/styles";
import { Drawer } from "@mui/material";
import { ReflexContainer, ReflexSplitter, ReflexElement } from "react-reflex";
import useMeasure from 'react-use-measure';
import useStore from "./stores/Store";
import { getTheme } from "./theme";
import "react-reflex/styles.css";
import "./App.css";
import shallow from "zustand/shallow";

export default function App() {
  const primaryColor = useStore((state) => state.primaryColor,shallow);
  const viewMode = useStore((state) => state.viewMode,shallow);
  const visibleSteps = useStore((state) =>
    state.focus.some((focusItem) =>
      TIMELINE_TYPES.includes(state.programData[focusItem]?.type)
    ),shallow
  );
  const setViewMode = useStore((state) => state.setViewMode,shallow);

  const [editorRef,editorBounds] = useMeasure();
  const [simRef,simBounds] = useMeasure();
  

  const theme = getTheme(primaryColor);
  const muiTheme = createTheme({
    palette: {
      mode: "dark",
      primaryColor: {
        main: primaryColor,
      },
      primary: {
        main: primaryColor,
      },
      quiet: {
        main: "#444",
        darker: "#333",
      },
    },
  });

  // const programRef = useRef();
  // const simulationRef = useRef();

  // const [open, setOpen] = useState(false);
  // console.log(viewMode);

  const showSim = viewMode === "default" || viewMode === "sim";
  const showEditor = viewMode === "default" || viewMode === "program";

  return (
    <Grommet full theme={theme}>
      {/* <CssBaseline/> */}
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
          {/* <Main open={open}> */}
          <Box fill direction="row" style={{paddingBottom:visibleSteps?'20vh':0}}>
            {/* <Box>
              <Box onClick={() => setOpen(!open)}>Bottom</Box>
            </Box> */}
            <ReviewTile />
            <ReflexContainer orientation="vertical">
              {showSim && (
                <ReflexElement
                  style={{}}
                  // minSize={200}
                  onStopResize={(e) => {
                    if (simBounds.width / editorBounds.width < 0.20) {
                      console.log('setting to program',e)
                      setViewMode("program");
                    }
                  }}
                >
                  <SimulatorTile ref={simRef}/>
                </ReflexElement>
              )}
              {viewMode === "default" && <ReflexSplitter />}

              {showEditor && (
                <ReflexElement
                  id='reflex-program'
                  style={{ overflow: "hidden" }}
                  // minSize={200}
                  onStopResize={(e) => {
                    if (editorBounds.width / simBounds.width < 0.20) {
                      console.log('setting to sim',e)
                      setViewMode("sim");
                    }
                  }}
                >
                  <ProgramTile ref={editorRef}/>
                </ReflexElement>
              )}
            </ReflexContainer>
          </Box>
          {/* </Main> */}

          <Drawer
            anchor="bottom"
            sx={{
              height: "20vh",
              flexShrink: 0,
              "& .MuiDrawer-paper": {
                height: "20vh",
                boxSizing: "border-box",
              },
            }}
            variant="persistent"
            open={visibleSteps}
          >
            <ParentSize>
              {({ width, height }) =>
                visibleSteps ? (
                  <TimelineGraph
                    width={width}
                    height={height - 10}
                    visible={visibleSteps}
                  />
                ) : null
              }
            </ParentSize>
          </Drawer>
          <Detail />
          <SettingsModal />
        </div>

        {/* <SettingsModal />
        
       */}
      </ThemeProvider>
    </Grommet>
  );
}

// const Content = () => (
//   <>
//     <motion.div
//       variants={mainVariants}
//       animate={visibleSteps ? "openDrawer" : "closedDrawer"}
//       style={{
//         flexDirection: "row",
//         display: "flex",
//       }}
//     >
//       <motion.div
//         variants={leftVariants}
//         animate={
//           viewMode === "default" || viewMode === "sim" ? "visible" : "hidden"
//         }
//         style={{
//           overflow: "hidden",
//         }}
//       ></motion.div>
//       <motion.div
//         layout
//         variants={rightVariants}
//         animate={
//           viewMode === "default" || viewMode === "program"
//             ? "visible"
//             : "hidden"
//         }
//         style={{
//           overflow: "hidden",
//         }}
//       >
//         <ProgramTile visible />
//       </motion.div>
//     </motion.div>
//     <motion.div
//       variants={drawerVariants}
//       animate={visibleSteps ? "openDrawer" : "closedDrawer"}
//       style={{
//         backgroundColor: "#444444",
//         borderTop: `5px solid ${primaryColor}`,
//       }}
//     >
//       <ParentSize>
//         {({ width, height }) =>
//           visibleSteps ? (
//             <TimelineGraph
//               width={width}
//               height={height - 10}
//               visible={visibleSteps}
//             />
//           ) : null
//         }
//       </ParentSize>
//     </motion.div>
//   </>
// );
