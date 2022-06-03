import React from "react";
import { Blurb } from "./Blurb";
import { Glossary } from "./Glossary";

export function getSkillInfo({ frame, primaryColor, focusItem }) {
  const content = (
    <div>
      <Blurb highlight="rgb(50,50,50)">
        <h3>About this Skill</h3>
        {focusItem.properties.description}
      </Blurb>
      <Blurb highlight="rgb(50,50,50)">
        <h3>What is a Skill?</h3>
        A Skill is a modular block of the program that can define its own
        parameters and execute other
        <Glossary.Actions primaryColor={primaryColor} /> and
        <Glossary.SkillCalls primaryColor={primaryColor} />.
      </Blurb>

      {frame === "quality" && (
        <Blurb highlight={primaryColor}>
          <h3 style={{ color: primaryColor }}>Program Quality</h3>
          Make sure to fully parameterize all
          <Glossary.Actions primaryColor={primaryColor} /> and
          <Glossary.SkillCalls primaryColor={primaryColor} /> in your skill.
        </Blurb>
      )}
    </div>
  );
  return content;
}
