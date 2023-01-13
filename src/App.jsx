import React, { useCallback } from "react";
// import { FiSettings } from "react-icons/fi";
import { ReviewTile } from "./components/Body/ReviewTile";
import { SimulatorTile } from "./components/Body/SimulatorTile";
import { ProgramTile } from "./components/Body/ProgramTile";
import ParentSize from "@visx/responsive/lib/components/ParentSize";
import { TIMELINE_TYPES, STATUS } from "./stores/Constants";
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
import {
  Drawer,
  Snackbar,
  Alert,
  AlertTitle,
  Stack,
} from "@mui/material";
import { ReflexContainer, ReflexSplitter, ReflexElement } from "react-reflex";
import useMeasure from "react-use-measure";
import useStore from "./stores/Store";
import useCompiledStore from "./stores/CompiledStore";
import "reactflow/dist/style.css";
import "react-reflex/styles.css";
import "./App.css";
import shallow from "zustand/shallow";

export default function App() {
  const primaryColor = useStore((state) => state.primaryColor, shallow);
  const viewMode = useStore((state) => state.viewMode, shallow);
  const visibleSteps = useStore(
    (state) =>
      state.focus.some((focusItem) =>
        TIMELINE_TYPES.includes(state.programData[focusItem]?.type)
      ),
    shallow
  );
  const focusData = useStore(
    (state) => state.focus.map((f) => state.programData[f]),
    shallow
  );

  const issueData = useStore((state) => {
    let issue = null;
    state.focus
      .slice()
      .reverse()
      .some((x) => {
        if (state.issues[x]) {
          issue = state.issues[x];
          return true;
        }
        return false;
      });
    return issue;
  }, shallow);

  const [focusSteps, errorType] = useCompiledStore(
    useCallback(
      (state) => {
        let steps = [];
        let errorType = null;
        if (!visibleSteps) {
          return [steps, errorType];
        }
        focusData.some((f) => {
          if (
            [STATUS.VALID, STATUS.PENDING, STATUS.WARN].includes(
              f?.properties?.status
            ) &&
            TIMELINE_TYPES.includes(f.type)
          ) {
            if (state[f.id] && Object.keys(state[f.id]).length === 1) {
              steps = state[f.id][Object.keys(state[f.id])[0]]?.steps;
              return true;
            } else {
              errorType = "traces";
              return false;
            }
          } else {
            errorType = "invalid";
            return false;
          }
        });
        return [steps, errorType];
      },
      [focusData, visibleSteps]
    ),
    shallow
  );

  const setViewMode = useStore((state) => state.setViewMode, shallow);
  const clearFocus = useStore((state) => state.clearFocus, shallow);

  const [editorRef, editorBounds] = useMeasure();
  const [simRef, simBounds] = useMeasure();

  // const theme = getTheme(primaryColor);
  const muiTheme = createTheme({
    palette: {
      mode: "dark",
      highlightColor: {
        main: primaryColor,
        darker: primaryColor,
      },
      primaryColor: {
        main: primaryColor,
        darker: primaryColor,
      },
      primary: {
        main: primaryColor,
      },
      quiet: {
        main: "#444",
        darker: "#333",
      },
      vibrant: {
        main: "#fff",
        darker: "#ddd",
      },
      safety: {
        main: "#CC79A7",
      },
      quality: {
        main: "#56B4E9",
      },
      performance: {
        main: "#E69F00",
      },
      business: {
        main: "#009E73",
      },
    },
    typography: {
      color:'white',
      fontFamily: [
        "-apple-system",
        "BlinkMacSystemFont",
        '"Segoe UI"',
        "Roboto",
        '"Helvetica Neue"',
        "Arial",
        "sans-serif",
        '"Apple Color Emoji"',
        '"Segoe UI Emoji"',
        '"Segoe UI Symbol"',
      ].join(","),
    },
  });

  // const programRef = useRef();
  // const simulationRef = useRef();

  // const [open, setOpen] = useState(false);
  // console.log(viewMode);

  const showSim = viewMode === "default" || viewMode === "sim";
  const showEditor = viewMode === "default" || viewMode === "program";

  return (
    // <Grommet full theme={theme}>
    <ThemeProvider theme={muiTheme}>
      <Stack
        direction="row"
        style={{
          backgroundColor: "red",
          height: "100vh",
          width: "100vw",
          position: "fixed",
        }}
      >
        <ReviewTile />
        <ReflexContainer
          orientation="vertical"
          style={{ backgroundColor: "blue" }}
        >
          {showSim && (
            <ReflexElement
              // minSize={200}
              onStopResize={(e) => {
                if (simBounds.width / editorBounds.width < 0.2) {
                  console.log("setting to program", e);
                  setViewMode("program");
                }
              }}
            >
              <SimulatorTile ref={simRef} />
            </ReflexElement>
          )}
          {viewMode === "default" && <ReflexSplitter />}

          {showEditor && (
            <ReflexElement
              id="reflex-program"
              style={{ overflow: "hidden"}}
              // minSize={200}
              onStopResize={(e) => {
                if (editorBounds.width / simBounds.width < 0.2) {
                  console.log("setting to sim", e);
                  setViewMode("sim");
                }
              }}
            >
              <ProgramTile ref={editorRef} />
            </ReflexElement>
          )}
        </ReflexContainer>
        <Snackbar open={errorType} autoHideDuration={6000} onClose={clearFocus}>
          <Alert
            variant="filled"
            severity="error"
            sx={{ width: "100%" }}
            onClose={clearFocus}
          >
            <AlertTitle>
              {errorType === "traces"
                ? "No single trace is available to display"
                : "Selected action contains errors"}
            </AlertTitle>
            {errorType === "traces" ? (
              <>
                <p>
                  This is usually because you are attempting to visualize an
                  action in a skill that is used multiple times.
                </p>
                <p>
                  To visualize, you will need to visualize the skill-call
                  instead.
                </p>
              </>
            ) : (
              <>
                <p>
                  You likely have not parameterized all fields correctly, or are
                  missing critical values.
                </p>
                <p>Consult the review panel for more suggestions.</p>
              </>
            )}
          </Alert>
        </Snackbar>
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
          open={visibleSteps && errorType === null}
        >
          <ParentSize>
            {({ width, height }) =>
              visibleSteps && errorType === null ? (
                <TimelineGraph
                  width={width}
                  height={height - 10}
                  focusSteps={focusSteps}
                  issue={
                    issueData?.graphData?.isTimeseries
                      ? issueData.graphData
                      : null
                  }
                />
              ) : null
            }
          </ParentSize>
        </Drawer>
        
      </Stack>
      <Detail />
      <SettingsModal />
      {/* <SettingsModal />
        
       */}
    </ThemeProvider>
    // </Grommet>
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
