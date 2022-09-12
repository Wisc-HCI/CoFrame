import React from "react";
// import { useSpring, animated } from "@react-spring/web";
import { motion } from "framer-motion";
// import { config, FrameValue } from "react-spring";
import frameStyles from "../frameStyles";
import { mapValues } from "lodash";

// const FRAMES = Object.keys(frameStyles.labels);

const GROUPS = {
  safety: (
    <g transform="matrix(1.02916,0,0,1.01786,-11.6651,-7.14347)">
      <path
        d="M400,792.982C353.449,789.798 197.071,683.881 147.213,629.294C97.355,574.707 50.655,163.972 91.441,127.699C132.228,91.426 243.561,92.86 276.22,79.059C308.88,65.259 358.74,7.018 400,7.018C441.26,7.018 491.12,65.259 523.78,79.059C556.439,92.86 667.772,91.426 708.559,127.699C749.345,163.972 702.645,574.707 652.787,629.294C602.929,683.881 446.551,789.798 400,792.982ZM337.333,337.333L249.579,337.333C232.286,337.333 218.246,351.373 218.246,368.667L218.246,431.333C218.246,448.627 232.286,462.667 249.579,462.667L337.333,462.667L337.333,550.421C337.333,567.714 351.373,581.754 368.667,581.754L431.333,581.754C448.627,581.754 462.667,567.714 462.667,550.421L462.667,462.667L550.421,462.667C567.714,462.667 581.754,448.627 581.754,431.333L581.754,368.667C581.754,351.373 567.714,337.333 550.421,337.333L462.667,337.333L462.667,249.579C462.667,232.286 448.627,218.246 431.333,218.246L368.667,218.246C351.373,218.246 337.333,232.286 337.333,249.579L337.333,337.333Z"
        // style="stroke:black;stroke-width:0.98px;"
      />
    </g>
  ),
  performance: (
    <g transform="matrix(1.07409,0,0,1.06898,-29.6361,7.59556e-15)">
      <path d="M347.286,55.593C347.228,54.704 347.199,53.808 347.199,52.906L347.199,41.084C347.199,18.409 365.608,-0 388.283,-0L406.082,-0C428.756,-0 447.165,18.409 447.165,41.084L447.165,52.906C447.165,53.536 447.151,54.163 447.123,54.789C617.141,77.817 748.377,223.703 748.377,400C748.377,592.275 592.275,748.377 400,748.377C207.725,748.377 51.623,592.275 51.623,400C51.623,315.092 82.064,237.237 132.612,176.774L125.106,167.983C110.346,150.698 112.396,124.681 129.682,109.921L143.252,98.333C160.538,83.574 186.554,85.624 201.314,102.91L207.233,109.841C248.566,82.312 296.1,63.377 347.286,55.593ZM365.828,406.931C365.709,412.055 368.029,417.151 372.491,420.379L490.375,505.657C497.58,510.87 507.661,509.252 512.874,502.046L531.762,475.937C536.974,468.731 535.356,458.65 528.151,453.438C528.151,453.438 435.013,391.429 431.895,382.512C427.714,370.553 430.275,162.881 430.275,162.881C430.275,153.989 423.055,146.769 414.162,146.769L381.937,146.769C373.044,146.769 365.824,153.989 365.824,162.881L365.824,406.551C365.824,406.678 365.825,406.805 365.828,406.931Z" />
    </g>
  ),
  quality: (
    <path d="M400,0C620.766,0 800,179.234 800,400C800,620.766 620.766,800 400,800C179.234,800 -0,620.766 -0,400C-0,179.234 179.234,0 400,0ZM367.757,134.887C372.82,121.472 385.661,112.593 400,112.593C414.339,112.593 427.18,121.472 432.243,134.887L487.416,281.07C488.032,282.704 489.114,284.122 490.527,285.149C491.94,286.175 493.623,286.766 495.368,286.847L651.445,294.147C665.769,294.817 678.181,304.285 682.611,317.922C687.042,331.56 682.567,346.515 671.372,355.476L549.394,453.122C548.03,454.213 547.016,455.68 546.476,457.341C545.936,459.003 545.894,460.785 546.356,462.47L587.645,613.164C591.434,626.993 586.264,641.724 574.663,650.152C563.063,658.58 547.456,658.945 535.475,651.068L404.915,565.233C403.455,564.274 401.747,563.762 400,563.762C398.253,563.762 396.545,564.274 395.085,565.233L264.525,651.068C252.544,658.945 236.937,658.58 225.337,650.152C213.736,641.724 208.566,626.993 212.355,613.164L253.644,462.47C254.106,460.785 254.064,459.003 253.524,457.341C252.984,455.68 251.97,454.213 250.606,453.122L128.628,355.476C117.433,346.515 112.958,331.56 117.389,317.922C121.819,304.285 134.231,294.817 148.555,294.147L304.632,286.847C306.377,286.766 308.06,286.175 309.473,285.149C310.886,284.122 311.968,282.704 312.584,281.07L367.757,134.887Z" />
  ),
  business: (
    <path d="M800,201.754C800,132.229 743.555,75.784 674.03,75.784L125.97,75.784C56.445,75.784 0,132.229 0,201.754L0,598.246C0,667.771 56.445,724.216 125.97,724.216L674.03,724.216C743.555,724.216 800,667.771 800,598.246L800,201.754ZM45.094,561.28L166.069,666.767C172.78,672.619 182.138,674.369 190.51,671.338L348.744,614.056C355.321,611.675 360.566,606.597 363.159,600.102L457.334,364.178C457.334,364.178 518.797,425.83 518.797,425.83C524.474,431.525 532.584,434.071 540.497,432.644C548.41,431.216 555.119,425.997 558.449,418.677L616.009,292.141C645.257,276.923 750.113,222.365 750.113,222.365C762.108,216.123 766.779,201.317 760.538,189.322C754.297,177.327 739.491,172.655 727.495,178.897L585.898,252.573C581.042,255.1 577.172,259.18 574.905,264.162L528.505,366.165C528.505,366.165 465.739,303.206 465.739,303.206C459.917,297.367 451.549,294.847 443.472,296.502C435.394,298.156 428.69,303.763 425.634,311.42L321.727,571.724C321.727,571.724 187.398,620.353 187.398,620.353C187.398,620.353 77.298,524.349 77.298,524.349C67.106,515.462 51.617,516.521 42.73,526.713C33.843,536.904 34.903,552.394 45.094,561.28Z" />
  ),
};

const Icon = ({ onChange, active, frame, width }) => {
  // const svgStyle = useSpring({
  //   fill: active ? "black" : frameStyles.colors[frame],
  //   color: active ? "black" : frameStyles.colors[frame],
  //   // config: config.molasses,
  // });
  const variants = {
    inactive: {backgroundColor:frameStyles.colors[frame],fill:frameStyles.colors[frame]},
    active: {backgroundColor:'#000000',fill:'#000000'},
  }

  const textVariants = {
    inactive: {color:frameStyles.colors[frame],fill:frameStyles.colors[frame]},
    active: {color:'#000000',fill:'#000000'},
  }

  // const spring = useSpring(active ? 0 : 1);
  // const color = useTransform(
  //   spring,
  //   [0, 1],
  //   ["black", frameStyles.colors[frame]]
  // );

  return (
    <div
      onClick={() => onChange(frame)}
      style={{
        unset: "all",
        zIndex: 100,
        display: "flex",
        flexDirection: "column",
        width,
      }}
    >
      <motion.svg
        layout
        variants={variants}
        animate={active ? 'active':'inactive'}
        width="100%"
        height="35px"
        viewBox="10 10 800 800"
        style={{
          unset: "all",
          fillRule: "evenodd",
          clipRule: "evenodd",
          strokeLinejoin: "round",
          strokeMiterlimit: 2,
          paddingTop: '2pt',
        }}
      >
        {GROUPS[frame]}
      </motion.svg>
      <motion.b
        layout
        variants={textVariants}
        animate={active ? 'active':'inactive'}
        style={{
          paddingTop: '2px',
          fontFamily: 'Helvetica',
          unset: "all",
          fontSize: 8,
          fontStyle: "bold",
          textAlign: "center",
        }}
      >
        {frameStyles.labels[frame]}
      </motion.b>
    </div>
  );
};

export const FrameTabBar = ({
  active,
  onChange,
  width = 335,
  backgroundColor = "black",
}) => {
  // const inactiveIconStyle

  const keys = Object.keys(frameStyles.colors);
  const variants = mapValues(frameStyles.colors, (v,k) => ({
    backgroundColor: v,
    x: keys.indexOf(k)*width/4,
  }));

  // console.log(variants)

  // const spring = useSpring(0);
  // const bcolor = useTransform(
  //   spring,
  //   [0, 1, 2, 3],
  //   ["safety", "quality", "performance", "business"].map(
  //     (v) => frameStyles.colors[v]
  //   )
  // );
  // const transformX = useTransform(
  //   spring,
  //   [0, 1, 2, 3],
  //   [0, 1, 2, 3].map((v) => (v * width) / 4)
  // );

  // console.log(transformX);
  // {
  //   backgroundColor: frameStyles.colors[active],
  //   translateX: FRAMES.indexOf(active) * width/4,
  //   // config: config.stiff,
  // });

  return (
    <div
      style={{
        color: "white",
        display: "flex",
        flexDirection: "row",
        height: 55,
        width,
        backgroundColor,
        position: "relative",
      }}
    >
      <motion.div
        variants={variants}
        animate={active}
        style={{
          // x: transformX,
          // backgroundColor: bcolor,
          width: width / 4,
          position: "absolute",
          height: '54px',
          borderRadius: 4,
        }}
      ></motion.div>
      <Icon
        frame="safety"
        active={active === "safety"}
        onChange={onChange}
        width={width / 4}
      />

      <Icon
        frame="quality"
        active={active === "quality"}
        onChange={onChange}
        width={width / 4}
      />

      <Icon
        frame="performance"
        active={active === "performance"}
        onChange={onChange}
        width={width / 4}
      />

      <Icon
        frame="business"
        active={active === "business"}
        onChange={onChange}
        width={width / 4}
      />
    </div>
  );
};
