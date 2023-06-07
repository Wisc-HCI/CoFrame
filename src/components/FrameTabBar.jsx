import React, { memo } from "react";
// import { useSpring, animated } from "@react-spring/web";
import { motion } from "framer-motion";
// import { config, FrameValue } from "react-spring";
import frameStyles from "../frameStyles";
import { mapValues } from "lodash";

export const FrameTabBar = memo(
  ({ active, onChange, width, height, vertical = false }) => {
    // const inactiveIconStyle

    const keys = Object.keys(frameStyles.colors).filter((k) => k !== "default");
    return (
      <motion.svg
        // layout
        width={width}
        height={height}
        // style={{ position: "relative" }}
        animate={vertical ? "v" : "h"}
        variants={{
          v: {
            width,
            height,
            viewBox: `0 0 ${800} ${800 * keys.length}`,
          },
          h: {
            width,
            height,
            viewBox: `0 0 ${800 * keys.length} ${1100}`,
          },
        }}
      >
        <motion.rect
          width={800}
          height={800}
          rx="200"
          animate={active}
          variants={mapValues(frameStyles.colors, (value, key) => ({
            x: vertical ? 0 : keys.indexOf(key) * 800,
            y: vertical ? keys.indexOf(key) * 800 : 0,
            fill: value,
          }))}
        />
        {keys.map((key) => (
          <g
            key={key}
            onClick={() => onChange(key)}
            transform={`translate(${
              vertical ? 80 : 80 + 800 * keys.indexOf(key)
            } ${vertical ? 80 + 800 * keys.indexOf(key) : 80}) scale(0.8 0.8)`}
          >
            <rect width={800} height={800} rx="200" fill="transparent" />
            <motion.g
              style={{
                unset: "all",
                fillRule: "evenodd",
                clipRule: "evenodd",
                strokeLinejoin: "round",
                strokeMiterlimit: 2,
                // paddingTop: "2pt",
              }}
              transform={frameStyles.icons[key].transform}
              variants={{
                inactive: {
                  backgroundColor: "transparent",
                  fill: frameStyles.colors[key],
                },
                active: { backgroundColor: "transparent", fill: "#000000" },
              }}
              animate={active === key ? "active" : "inactive"}
            >
              <path d={frameStyles.icons[key].path} />
            </motion.g>
          </g>
        ))}
        {!vertical && (
          <motion.text
            layout
            variants={mapValues(frameStyles.colors, (value) => ({
              color: value,
              fill: value,
            }))}
            animate={active}
            x={400 * keys.length}
            y={1000}
            fontSize={250}
            fontFamily="Helvetica"
            textAnchor="middle"
          >
            {frameStyles.labels[active]}
          </motion.text>
        )}
      </motion.svg>
    );
  }
);
