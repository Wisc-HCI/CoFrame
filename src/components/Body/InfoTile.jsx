import React from "react";
import { Box, Button } from "grommet";
import useStore from "../../stores/Store";
import shallow from "zustand/shallow";
import { FiChevronRight } from "react-icons/fi";
import { getLocationInfo } from "../ContextualInfo/LocationBlock";
import { getRobotAgentInfo } from "../ContextualInfo/RobotAgentBlock";
import { getToolInfo } from "../ContextualInfo/ToolBlock";
import { getFixtureInfo } from "../ContextualInfo/FixtureBlock";
import { getProcessInfo } from "../ContextualInfo/ProcessBlock";
import { getWaypointInfo } from "../ContextualInfo/WaypointBlock";
import { getMachineInfo } from "../ContextualInfo/MachineBlock";
import { getThingInfo } from "../ContextualInfo/ThingBlock";
import { getProgramInfo } from "../ContextualInfo/ProgramBlock";
import { getSkillInfo } from "../ContextualInfo/SkillBlock";
import { getPrimitiveInfo } from "../ContextualInfo/PrimitiveBlock";
import { getTrajectoryInfo } from "../ContextualInfo/TrajectoryBlock";
import { getInputOutputInfo } from "../ContextualInfo/InputOutputBlock";
import { getGripperInfo } from "../ContextualInfo/GripperBlock";
import { getIssueInfo } from "../ContextualInfo/Issue";
import Tile from "../Elements/Tile";
import { DATA_TYPES } from "simple-vp";
import actionTypes from "../../stores/typeInfo/action";
import { stringEquality } from "../../helpers/performance";
import { Breadcrumbs } from "@mui/material";
import { ScrollRegion } from "../Elements/ScrollRegion";

export function InfoTile({ maxHeight }) {
  const [frame, primaryColor, focusData, activeFocus, setActiveFocus] =
    useStore((state) => {
      let focusData = state.focus
        .map((f) => {
          if (state.programData[f]) {
            return state.programData[f];
          } else if (state.issues[f]) {
            return state.issues[f];
          } else {
            return null;
          }
        })
        .filter((d) => d !== null);

      let programData = null;
      Object.values(state.programData).some((d) => {
        if (d.dataType === DATA_TYPES.INSTANCE && d.type === "programType") {
          programData = d;
          return true;
        } else {
          return false;
        }
      });

      if (focusData.length === 0) {
        focusData = [programData];
      }

      return [
        state.frame,
        state.primaryColor,
        focusData,
        state.activeFocus,
        state.setActiveFocus,
      ];
    }, stringEquality);

  // const [currentTab, setCurrentTab] = useState(focusData.length-1);

  let tabs = focusData.map((focusItem, i) => {
    if (focusItem.code) {
      // Is an issue
      return {
        title: focusItem.title,
        key: focusItem.id,
        contents: getIssueInfo({ frame, primaryColor, focusItem }),
      };
    } else if (focusItem.type !== undefined) {
      let contents = <div>DATA CONTENT</div>;
      if (focusItem.type === "programType") {
        contents = getProgramInfo({
          frame,
          primaryColor,
          focusItem,
        });
      } else if (Object.keys(actionTypes).includes(focusItem.type)) {
        contents = getPrimitiveInfo({
          frame,
          primaryColor,
          focusItem,
        });
      } else if (focusItem.type === "locationType") {
        contents = getLocationInfo({ frame, primaryColor, focusItem });
      } else if (focusItem.type === "waypointType") {
        contents = getWaypointInfo({ frame, primaryColor, focusItem });
      } else if (focusItem.type === "machineType") {
        contents = getMachineInfo({ frame, primaryColor, focusItem });
      } else if (focusItem.type === "thingType") {
        contents = getThingInfo({ frame, primaryColor, focusItem });
      } else if (focusItem.type === "skillType") {
        contents = getSkillInfo({ frame, primaryColor, focusItem });
      } else if (focusItem.type === "trajectoryType") {
        contents = getTrajectoryInfo({ frame, primaryColor, focusItem });
      } else if (focusItem.type === "processType") {
        contents = getProcessInfo({ frame, primaryColor, focusItem });
      } else if (focusItem.type === "fixtureType") {
        contents = getFixtureInfo({ frame, primaryColor, focusItem });
      } else if (focusItem.type === "toolType") {
        contents = getToolInfo({ frame, primaryColor, focusItem });
      } else if (focusItem.type === "robotAgentType") {
        contents = getRobotAgentInfo({ frame, primaryColor, focusItem });
      } else if (focusItem.type === "inputOutputType") {
        contents = getInputOutputInfo({ frame, primaryColor, focusItem });
      } else if (focusItem.type === "gripperType") {
        contents = getGripperInfo({ frame, primaryColor, focusItem });
      }
      // Is a block/data object
      return { title: focusItem.name, key: focusItem.id, contents };
    } else {
      return { title: "null", key: i, contents: <div>NULL CONTENT</div> };
    }
  });

  let tabIdx = 0;
  tabs.some((tab, i) => {
    if (tab.key === activeFocus) {
      tabIdx = i;
      return true;
    } else {
      return false;
    }
  });

  //   const currentIssue = undefined;
  //   const description = useStore(
  //     (state) =>
  //       state.programData[focusData[focusData.length - 1].id]?.properties
  //         .description
  //   );

  //   const issueParams = {
  //     activeDrawer,
  //     frame,
  //     primaryColor,
  //     focusData,
  //     currentIssue,
  //     description,
  //   };
  //   if (focusData[focusData.length - 1].type === "locationType") {
  //     tabs = getLocationInfo(issueParams);
  //   } else if (focusData[focusData.length - 1].type === "waypointType") {
  //     tabs = getWaypointInfo(issueParams);
  //   } else if (focusData[focusData.length - 1].type === "machineType") {
  //     tabs = getMachineInfo(issueParams);
  //   } else if (focusData[focusData.length - 1].type === "thingType") {
  //     tabs = getThingInfo(issueParams);
  //   } else if (focusData[focusData.length - 1].type === "skillType") {
  //     tabs = getSkillInfo(issueParams);
  //   } else if (focusData[focusData.length - 1].type === "trajectoryType") {
  //     tabs = getTrajectoryInfo(issueParams);
  //   } else if (focusData[focusData.length - 1].type === "processType") {
  //     tabs = getProcessInfo(issueParams);
  //   } else if (focusData[focusData.length - 1].type === "fixtureType") {
  //     tabs = getFixtureInfo(issueParams);
  //   } else if (focusData[focusData.length - 1].type === "toolType") {
  //     tabs = getToolInfo(issueParams);
  //   } else if (focusData[focusData.length - 1].type === "robotAgentType") {
  //     tabs = getRobotAgentInfo(issueParams);
  //   } else if (focusData[focusData.length - 1].type === "inputOutputType") {
  //     tabs = getInputOutputInfo(issueParams);
  //   } else if (focusData[focusData.length - 1].type === "gripperType") {
  //     tabs = getGripperInfo(issueParams);
  //   } else if (
  //     !focusData[focusData.length - 1].type &&
  //     focusData[focusData.length - 1].graphData
  //   ) {
  //     tabs = getPlotInfo({ currentIssue: focusData[focusData.length - 1] });
  //   }

  //     tabs = getThingInfo(issueParams)
  // } else if (focusData?.type === 'program' || activeDrawer === null) {
  //     tabs = getProgramInfo(issueParams)
  // } else if (focusData?.type === 'skill') {
  //     tabs = getSkillInfo(issueParams)
  // } else if (focusData?.type === 'primitive') {
  //     tabs = getPrimitiveInfo(issueParams)
  // } else if (focusData?.type === 'trajectory') {
  //     tabs = getTrajectoryInfo(issueParams)
  // }

  // if (currentIssue && currentIssue.graphData) {
  //     let temp = getPlotInfo(issueParams);
  //     tabs = tabs.concat(temp);
  // }

  // console.log(tabs);

  return (
    <Tile
      style={{ display: "flex", flexDirection: "column", height: maxHeight,overflow:'hidden' }}
      borderRadius={0}
      borderWidth={4}
      internalPaddingWidth={0}
      header={
        <Breadcrumbs separator={<FiChevronRight/>}>
          {tabs.map((tab, i) => (
            // <Box key={i} direction="row" align="center" alignContent="center">
              <Button
              key={i}
                plain
                label={tab.title}
                onClick={() => setActiveFocus(tab.key)}
                margin={{
                  top: "7pt",
                  bottom: "7pt",
                  right: "5pt",
                  left: "5pt",
                }}
                style={{
                  color: tab.key === activeFocus ? primaryColor : "white",
                }}
              />
              // {i < tabs.length - 1 && <FiChevronRight />}
            // </Box>
          ))}
        </Breadcrumbs>
      }
    >
      <ScrollRegion vertical height={maxHeight-48}>
        <Box fill pad={{right:'small',left:'xsmall',top:'xsmall',bottom:'xsmall'}}>
          {tabs[tabIdx] ? tabs[tabIdx].contents : tabs[0].contents}
        </Box>
        
      </ScrollRegion>
      {/* <div style={{ height: maxHeight - 68, overflowY: "scroll" }}>
        
      </div> */}
    </Tile>
  );
}
