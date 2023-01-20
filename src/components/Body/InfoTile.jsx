import React, { memo } from "react";
import useStore from "../../stores/Store";
import Tile from "../Elements/Tile";
import { stringEquality } from "../../helpers/performance";
import { Stack, Typography } from "@mui/material";
import { ScrollRegion } from "../Elements/ScrollRegion";
import { GoalSection } from "../Goal/GoalSection";

export const InfoTile = memo(({ maxHeight }) => {
  const goalList =
    useStore((state) => Object.values(state.programData).filter(v => v.type === "goalType"), stringEquality);

  return (
    <Tile
      style={{ display: "flex", flexDirection: "column", height: maxHeight,overflow:'hidden' }}
      borderRadius={0}
      borderWidth={4}
      internalPaddingWidth={0}
      header={<Typography style={{ margin: "10pt", color:'white'}}>Task Goals</Typography>}
    >
      <ScrollRegion vertical height={maxHeight-55}>
        <Stack style={{ paddingLeft: 10, paddingTop: 10, paddingRight: 15}}>
          <GoalSection goalList={goalList}/>
        </Stack>
      </ScrollRegion>
    </Tile>
  );
})
