import React from 'react';
import { useState } from "react";
import { useSpring, animated } from "@react-spring/web";
import { config } from "react-spring";

export const Toggle = ({
  size,
  selected,
  disabled,
  selectedText,
  deselectedText,
  backgroundColor,
  onClick
}) => {
  const sizeMultiplier = size === "small" ? 1 : 1.5;

  const [toggle, setToggle] = useState(false);

  const { x, ...markerStyle } = useSpring({
    width: sizeMultiplier * 15,
    height: sizeMultiplier * 15,
    left: disabled ? null : selected ? sizeMultiplier * 22 : sizeMultiplier * 3,
    x: toggle ? 0 : 1,
    backgroundColor: selected ? "white" : "lightgrey",
    config: config.stiff
  });

  const buttonStyle = useSpring({
    backgroundColor: selected ? backgroundColor : "rgb(145,145,145)",
    width: sizeMultiplier * 40,
    height: sizeMultiplier * 20,
    config: config.stiff
  });

  const textStyle = useSpring({
    fontSize: sizeMultiplier * 6,
    color: selected ? "white" : "lightgrey",
    transform: `translate3d(${
      selected ? -12 * sizeMultiplier : -1 * sizeMultiplier
    }pt,-${8.5 * sizeMultiplier}pt,0pt)`,
    config: config.stiff
  });

  const markerOffset = selected ? 22 : 3;

  return (
    <animated.button
      onClick={
        disabled ? () => setToggle(!toggle) : onClick ? onClick : () => {}
      }
      style={{
        ...buttonStyle,
        borderRadius: 100,
        borderWidth: 0,
        boxShadow: "inset 0.5pt 0.5pt 0pt 0pt rgba(55,55,55,0.25)",
        textAlign: "center",
        display: "inline-block",
        margin: 0,
        padding: 0
      }}
    >
      <animated.div
        style={{
          ...markerStyle,
          borderRadius: 100,
          left: disabled
            ? x.to({
                range: [0, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 1],
                output: [0, -1.5, -5, 5, -5, 5, 1.5, 0].map(
                  (v) => (v * 0.5 + markerOffset) * sizeMultiplier
                )
              })
            : markerStyle.left,
          top: 0,
          position: "relative",
          boxShadow: "0.5pt 0.5pt 0pt 0pt rgba(55,55,55,0.25)"
        }}
      />
      <animated.span
        style={{
          position: "absolute",
          textShadow: "0.5pt 0.5pt 0pt 0pt rgba(55,55,55,0.25)",
          zIndex: 10,
          ...textStyle
        }}
      >
        {selected ? selectedText : deselectedText}
      </animated.span>
    </animated.button>
  );
};