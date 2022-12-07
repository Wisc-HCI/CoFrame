import React from "react";
import useStore from "../../stores/Store";
import { FiChevronRight } from "react-icons/fi";
import { getIssueInfo } from "../ContextualInfo/Issue";
import Tile from "../Elements/Tile";
import { DATA_TYPES } from "simple-vp";
import { stringEquality } from "../../helpers/performance";
import { Breadcrumbs, Stack, Button, Chip } from "@mui/material";
import { ScrollRegion } from "../Elements/ScrollRegion";
import { GoalSection } from "../Goal/GoalSection";
import { emphasize, styled } from '@mui/material/styles';

const StyledBreadcrumb = styled(Chip)(({ theme }) => {
  const backgroundColor =
    theme.palette.mode === 'light'
      ? theme.palette.grey[100]
      : theme.palette.grey[800];
  return {
    backgroundColor,
    height: theme.spacing(3),
    color: theme.palette.text.primary,
    fontWeight: theme.typography.fontWeightRegular,
    '&:hover, &:focus': {
      backgroundColor: emphasize(backgroundColor, 0.06),
    },
    '&:active': {
      boxShadow: theme.shadows[1],
      backgroundColor: emphasize(backgroundColor, 0.12),
    },
  };
})

export function InfoTile({ maxHeight }) {
  const [frame, primaryColor, focusData, activeFocus, setActiveFocus, goalList] =
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
        Object.values(state.programData).filter(v => v.type === "goalType")
      ];
    }, stringEquality);

  // const [currentTab, setCurrentTab] = useState(focusData.length-1);

  let tabs = [];
  
  if (goalList.length > 0) {
    tabs.push({
      title: "Task Goals",
      key: "task-goals",
      contents: <GoalSection goalList={goalList}/>,
    })
  }

  focusData.forEach((focusItem, i) => {
    if (focusItem.code) {
      // Is an issue
      tabs.push({
        title: focusItem.title,
        key: focusItem.id,
        contents: getIssueInfo({ frame, primaryColor, focusItem }),
      });
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

  return (
    <Tile
      style={{ display: "flex", flexDirection: "column", height: maxHeight,overflow:'hidden' }}
      borderRadius={0}
      borderWidth={4}
      internalPaddingWidth={0}
      header={
        <Breadcrumbs separator={<FiChevronRight/>}>
          {tabs.map((tab, i) => (
              <StyledBreadcrumb
                key={i}
                label={tab.title}
                onClick={() => setActiveFocus(tab.key)}
                style={{
                  color: tab.key === activeFocus ? primaryColor : "white",
                }}
              />
          ))}
        </Breadcrumbs>
      }
    >
      <ScrollRegion vertical height={maxHeight-30}>
        <Stack style={{ paddingLeft: 10, paddingTop: 10, paddingRight: 15}}>
          {tabs[tabIdx] ? tabs[tabIdx].contents : tabs[0].contents}
        </Stack>
      </ScrollRegion>
    </Tile>
  );
}
