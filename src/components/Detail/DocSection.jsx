import React, { useCallback, useState, useEffect } from "react";
import useStore from "../../stores/Store";
import {
  TYPES,
  DATA_TYPES,
  SIMPLE_PROPERTY_TYPES
} from "simple-vp";
import { shallow } from 'zustand/shallow';
import { Collapse } from "../Elements/Collapse";
import {
  Avatar,
  Card,
  CardHeader,
  CardContent,
  Tooltip,
  Chip,
  Tabs,
  Tab,
  Box,
  Breadcrumbs,
  Typography,
  Badge,
  Divider,
  Alert,
  Accordion,
  AccordionDetails,
  AccordionSummary,
  Stack,
  Grid
} from "@mui/material";
import { darken, emphasize, styled } from "@mui/material/styles";
import { pickBy, mapValues } from "lodash";
import {
  FiChevronDown,
  FiCircle,
  FiCode,
  FiLogIn,
  FiList,
  FiLogOut,
  FiPaperclip,
  FiStar,
  FiDownload,
  FiSquare,
} from "react-icons/fi";
import { Remark } from "react-remark";

export const DocSection = ({ data }) => {
  const [path, setPath] = useState([data.type]);

  useEffect(()=>{setPath([data.type])},[data]);

  const activeType = path[path.length - 1];
  const typeInfo = useStore(
    (state) =>
      functionTypeSpec(state.programSpec.objectTypes, state.programData),
    shallow
  );
  const haloColor = darken(getColor(typeInfo[activeType]), 0.5);

  const featuredDoc = useStore(
    (state) =>
      typeof state.featuredDocs[data.id] === "string"
        ? state.featuredDocs[data.id]
        : typeof state.featuredDocs[data.refData?.id] === "string"
        ? state.featuredDocs[data.refData?.id]
        : null,
    shallow
  );
  const tabs = featuredDoc
    ? ["featured", "description", "usage"]
    : ["description", "usage"];
  const [tab, setTab] = useState(tabs[0]);

  const references = getReferences(typeInfo, activeType);

  const connections = ["instanceBlock", "referenceBlock", "callBlock"].filter(
    (blockType) =>
      typeInfo[activeType]?.[blockType]?.onCanvas &&
      typeInfo[activeType]?.[blockType]?.connections
  );

  const handleLinkClick = (value) => {
    if (path.includes(value)) {
      let found = false;
      const newPath = path.filter((item) => {
        if (item === value) {
          found = true;
          return true;
        } else if (item !== value && found) {
          return false;
        } else if (item !== value) {
          return true;
        }
      });
      setPath(newPath);
    } else {
      setPath([...path, value]);
    }
  };

  return (
    <Collapse
      header={
        <Badge badgeContent="!" color="primaryColor" invisible={!featuredDoc}>
          Information
        </Badge>
      }
      style={{ boxShadow: data.type === activeType ? null : `inset 0px 0px 0px 2px ${haloColor}` }}
    >
      <>
        <Tabs
          value={tab}
          sx={{
            backgroundColor: haloColor,
            borderTopLeftRadius: 4,
            borderTopRightRadius: 4,
          }}
          onChange={(_, tab) => setTab(tab)}
          indicatorColor="primary"
          textColor="inherit"
          variant="fullWidth"
          aria-label="doc-section-tabs"
        >
          {tabs.map((tab) => (
            <Tab key={tab} label={tab} value={tab} />
          ))}
        </Tabs>
        {(tab === "description" || tab === "usage") && (
          <Box style={{ padding: 2, backgroundColor: "#252525" }}>
            <Breadcrumbs>
              {path.map((item) => (
                <StyledBreadcrumb
                  key={item}
                  underline="hover"
                  label={typeInfo[item]?.name || "Unrecognized"}
                  onClick={(e) => {
                    e.preventDefault();
                    handleLinkClick(item);
                  }}
                  onDelete={(e) => {
                    e.preventDefault();
                    handleLinkClick(item);
                  }}
                  deleteIcon={
                    <FiCircle
                      style={{
                        height: 13,
                        width: 13,
                        fill: getColor(typeInfo[item]),
                      }}
                    />
                  }
                />
              ))}
            </Breadcrumbs>
          </Box>
        )}
        {tab === "featured" && (
          <Remark
            key='featured'
            rehypeReactOptions={{
              components: componentLookup(handleLinkClick,typeInfo,tab),
            }}
          >
            {featuredDoc}
          </Remark>
        )}
        {tab === "description" && (
          <Remark
            key='description'
            rehypeReactOptions={{
              components: componentLookup(handleLinkClick,typeInfo,tab),
            }}
          >
            {typeInfo[activeType]?.description || "No Description"}
          </Remark>
        )}
        {tab === "usage" && (
          <>
            <Accordion key='fields' sx={{ backgroundColor: "#333" }}>
              <AccordionSummary
                expandIcon={<FiChevronDown />}
                aria-controls="panel1a-content"
                id="panel1a-header"
              >
                <Typography>
                  <b>
                    <i>{typeInfo[activeType]?.name || "Unrecognized"}</i>
                  </b>{" "}
                  Fields
                </Typography>
              </AccordionSummary>
              <AccordionDetails sx={{ backgroundColor: "#2D2D2D", padding: 1 }}>
                {Object.keys(typeInfo[activeType]?.properties || {}).length ===
                  0 && (
                  <Typography
                    style={{ textAlign: "center" }}
                    color="text.secondary"
                  >
                    This block has no fields
                  </Typography>
                )}
                <Stack spacing={1}>
                  {Object.keys(typeInfo[activeType]?.properties || {}).map(
                    (key) => (
                      <FieldInfo
                        key={key}
                        parent={activeType}
                        field={key}
                        typeInfo={typeInfo}
                        handleLinkClick={handleLinkClick}
                      />
                    )
                  )}
                </Stack>
              </AccordionDetails>
            </Accordion>
            <Accordion key='asField' sx={{ backgroundColor: "#333" }}>
              <AccordionSummary
                expandIcon={<FiChevronDown />}
                aria-controls="panel1a-content"
                id="panel1a-header"
              >
                <Typography>
                  <b>
                    <i>{typeInfo[activeType]?.name || "Unrecognized"}</i>
                  </b>{" "}
                  as a Field
                </Typography>
              </AccordionSummary>
              <AccordionDetails sx={{ backgroundColor: "#2D2D2D", padding: 1 }}>
                <Stack spacing={1}>
                  {references.length === 0 && (
                    <Typography
                      style={{ textAlign: "center" }}
                      color="text.secondary"
                    >
                      This block is not used anywhere
                    </Typography>
                  )}
                  {references.map((block) => (
                    <>
                      <Divider>
                        <TypeLink
                          label={typeInfo[block.parent]?.name}
                          color={getColor(typeInfo[block.parent])}
                          onClick={(e) => {
                            e.preventDefault();
                            if (typeInfo[block.parent]) {
                              handleLinkClick(block.parent);
                            }
                          }}
                        />
                      </Divider>
                      {block.fields.map((field) => (
                        <FieldInfo
                          key={`${block.parent}-${field}`}
                          parent={block.parent}
                          field={field}
                          typeInfo={typeInfo}
                          handleLinkClick={handleLinkClick}
                        />
                      ))}
                    </>
                  ))}
                </Stack>
              </AccordionDetails>
            </Accordion>
            <Accordion key='connections' sx={{ backgroundColor: "#333" }}>
              <AccordionSummary
                expandIcon={<FiChevronDown />}
                aria-controls="panel1a-content"
                id="panel1a-header"
              >
                <Typography>Connections</Typography>
              </AccordionSummary>
              <AccordionDetails sx={{ backgroundColor: "#2D2D2D", padding: 1 }}>
                {connections.length > 0 ? (
                  connections.map((blockType) => (
                    <React.Fragment key={blockType}>
                      <Divider>
                        <span
                          style={{
                            fontFamily: "Helvetica",
                            textTransform: "capitalize",
                          }}
                        >
                          {blockType.replace("Block", "")}
                        </span>
                      </Divider>
                      {Object.entries(
                        typeInfo[activeType][blockType].connections
                      ).map(([side, connectInfo]) => (
                        <ConnectionInfo
                          key={`${side}-${blockType}`}
                          side={side}
                          connectionInfo={connectInfo}
                          typeInfo={typeInfo}
                          handleLinkClick={handleLinkClick}
                        />
                      ))}
                    </React.Fragment>
                  ))
                ) : (
                  <Typography
                    style={{ textAlign: "center" }}
                    color="text.secondary"
                  >
                    This block has no connections
                  </Typography>
                )}
              </AccordionDetails>
            </Accordion>
          </>
        )}
      </>
    </Collapse>
  );
};

const functionTypeSpec = (typeSpec, programData) => {
  const augmented = pickBy(typeSpec, (info) => info.type !== TYPES.FUNCTION);
  const functionKeys = Object.keys(typeSpec).filter(
    (typeKey) => typeSpec[typeKey].type === TYPES.FUNCTION
  );
  Object.values(programData)
    .filter(
      (data) =>
        data.dataType === DATA_TYPES.INSTANCE &&
        functionKeys.includes(data.type)
    )
    .forEach((functionInstance) => {
      const initialFunctionDef = {
        ...typeSpec[functionInstance.type],
        name: functionInstance.name,
      };
      let newProperties = {};
      functionInstance.arguments.forEach((arg) => {
        const argBlock = programData[arg];
        newProperties[arg] = {
          name: argBlock.name,
          accepts: [argBlock.type],
          default: null,
          isFunctionArgument: true,
        };
      });
      let functionTypeDef = {
        ...initialFunctionDef,
        properties: {
          ...mapValues(initialFunctionDef.properties, (v) => ({
            ...v,
            isFunctionBlockField: true,
          })),
          ...newProperties,
        },
      };
      augmented[functionInstance.id] = functionTypeDef;
    });
  return augmented;
};

const StyledBreadcrumb = styled(Chip)(({ theme }) => {
  const backgroundColor =
    theme.palette.mode === "light"
      ? theme.palette.grey[100]
      : theme.palette.grey[800];
  return {
    backgroundColor,
    height: theme.spacing(3),
    color: theme.palette.text.primary,
    fontWeight: theme.typography.fontWeightRegular,
    "&:hover, &:focus": {
      backgroundColor: emphasize(backgroundColor, 0.06),
    },
    "&:active": {
      boxShadow: theme.shadows[1],
      backgroundColor: emphasize(backgroundColor, 0.12),
    },
  };
});

const componentLookup = (handleLinkClick,typeInfo,tab) => ({
  h1: ({ node, ...props }) => <Typography variant="h3" {...props} />,
  h2: ({ node, ...props }) => <Typography variant="h4" {...props} />,
  h3: ({ node, ...props }) => <Typography variant="h5" {...props} />,
  h4: ({ node, ...props }) => <Typography variant="h6" {...props} />,
  h5: ({ node, ...props }) => <Typography variant="body1" {...props} />,
  h6: ({ node, ...props }) => <Typography variant="body1" {...props} />,
  p: ({ node, ...props }) => <Typography variant="body1" {...props} />,
  a: ({ node, ...props }) => {
    return (
      <TypeLink
        label={props.children[0]}
        color={getColor(typeInfo[props.href])}
        onClick={(e) => {
          e.preventDefault();
          if (typeInfo[props.href]) {
            handleLinkClick(props.href);
            if (tab === "featured" && props.href !== data.type) {
              setTab("description");
            }
          }
        }}
      />
    );
  },
  code: ({ node, inline, className, children, ...props }) => {
    const match = /language-(\w+)/.exec(className || "");
    return !inline && match ? (
      <SyntaxHighlighter
        children={String(children).replace(/\n$/, "")}
        style={oneDark}
        language={match[1]}
        // PreTag="pre"
        {...props}
      />
    ) : (
      <Box
        style={{
          backgroundColor: "#444",
          borderRadius: 4,
          padding: 5,
        }}
      >
        <code className={className} {...props}>
          {children}
        </code>
      </Box>
    );
  },
  ol: ({ node, ordered, ...props }) => (
    <ol
      {...props}
      style={{
        fontFamily: "helvetica",
        backgroundColor: "#44444444",
        borderRadius: 4,
        paddingTop: 5,
        paddingBottom: 5,
      }}
    />
  ),
  ul: ({ node, ordered, ...props }) => (
    <ul
      {...props}
      style={{
        fontFamily: "helvetica",
        backgroundColor: "#44444480",
        borderRadius: 4,
        paddingTop: 5,
        paddingBottom: 5,
      }}
    />
  ),
  blockquote: ({ node, ...props }) => {
    let severity = "info";
    let color = "quiet";
    let cleaned = [];
    props.children.forEach((child) => {
      if (typeof child === "object" && child.props?.children) {
        child.props.children.forEach((innerChild, idx) => {
          if (typeof innerChild === "string" && innerChild.includes("[info]")) {
            severity = "info";
            color = "info";
            child.props.children[idx] = innerChild.replace("[info]", "");
          } else if (
            typeof innerChild === "string" &&
            innerChild.includes("[success]")
          ) {
            severity = "success";
            color = "success";
            child.props.children[idx] = innerChild.replace("[success]", "");
          } else if (
            typeof innerChild === "string" &&
            innerChild.includes("[warn]")
          ) {
            severity = "warning";
            color = "warning";
            child.props.children[idx] = innerChild.replace("[warn]", "");
          } else if (
            typeof innerChild === "string" &&
            innerChild.includes("[error]")
          ) {
            severity = "error";
            color = "error";
            child.props.children[idx] = innerChild.replace("[error]", "");
          } else {
          }
        });
      }
      cleaned.push(child);
    });
    return (
      <Alert
        variant="filled"
        severity={severity}
        color={color}
        icon={color === "quiet" ? <FiPaperclip /> : undefined}
      >
        <span children={cleaned} />
      </Alert>
    );
  },
});

const getColor = (spec) => {
  return (
    spec?.instanceBlock?.color ||
    spec?.referenceBlock?.color ||
    spec?.callBlock?.color ||
    "#bbb"
  );
};

const ChipMimic = styled("button")(({ theme }) => {
  const backgroundColor =
    theme.palette.mode === "light"
      ? theme.palette.grey[100]
      : theme.palette.grey[800];
  return {
    display: "inline-block",
    backgroundColor,
    // paddingTop: 3,
    // paddingBottom: 3,
    paddingLeft: 6,
    paddingRight: 4,
    borderRadius: 100,
    fontSize: 14,
    border: "unset",
    height: theme.spacing(3),
    alignItems: "center",
    alignContent: "center",
    color: theme.palette.text.primary,
    fontWeight: theme.typography.fontWeightRegular,
    "&:hover, &:focus": {
      backgroundColor: emphasize(backgroundColor, 0.06),
    },
  };
});

const TypeLink = ({ label, color, onClick }) => {
  return (
    <ChipMimic onClick={onClick}>
      {label}{" "}
      <FiCircle
        style={{
          position: "relative",
          alignSelf: "center",
          height: 15,
          width: 15,
          top: 2.5,
          //   bottom:-3,
          fill: color,
        }}
      />
      <span
        style={{
          height: 15,
          width: 15,
          borderRadius: 100,
          backgroundColor: color,
          border: 1,
          borderColor: "white",
        }}
      >
        {" "}
      </span>
    </ChipMimic>
  );
};

const getReferences = (typeSpec, usedType) => {
  let references = [];
  Object.keys(typeSpec).forEach((typeValue) => {
    let entry = { parent: typeValue, fields: [] };
    if (typeSpec[typeValue].properties) {
      Object.keys(typeSpec[typeValue].properties).forEach((prop) => {
        if (typeSpec[typeValue].properties[prop].accepts?.includes(usedType)) {
          entry.fields.push(prop);
        }
      });
    }
    if (entry.fields.length > 0) {
      references.push(entry);
    }
  });
  return references;
};

export const TypeDescription = ({ type }) => {
  const info = useStore(
    useCallback((state) => state.programSpec.objectTypes[type], [type]),
    shallow
  );
  return (
    <TypeLink label={info?.name || "Unknown Block"} color={getColor(info)} />
  );
};

const SHOWN_SIMPLE_TYPES = [
  SIMPLE_PROPERTY_TYPES.BOOLEAN,
  SIMPLE_PROPERTY_TYPES.NUMBER,
  SIMPLE_PROPERTY_TYPES.STRING,
  SIMPLE_PROPERTY_TYPES.OPTIONS,
  SIMPLE_PROPERTY_TYPES.VECTOR3,
];

const FieldInfo = ({ parent, field, typeInfo, handleLinkClick }) => {
  const fieldInfo = typeInfo[parent]?.properties?.[field];
  const variant = fieldInfo?.accepts
    ? "block"
    : SHOWN_SIMPLE_TYPES.includes(fieldInfo?.type)
    ? "simple"
    : "na";

  if (variant === "na") {
    return null;
  }
  // console.log('info',{field,fieldInfo,typeInfo})
  return (
    <Card sx={{ padding: 2 }}>
      <CardHeader
        variant="h5"
        color="text.primary"
        title={fieldInfo.name}
        sx={{ padding: 0 }}
        action={
          fieldInfo.accepts ? (
            <Stack direction="row" gap={0.5}>
              {fieldInfo.isList && (
                <Tooltip
                  key='isList'
                  title="This property accepts a set of entries as a list"
                  sx={{ fontSize: 20 }}
                  arrow
                >
                  <Avatar
                    sx={{
                      width: 30,
                      height: 30,
                      backgroundColor: DOC_FLAG_COLORS.IS_LIST,
                    }}
                  >
                    <FiList />
                  </Avatar>
                </Tooltip>
              )}
              {fieldInfo.fullWidth && (
                <Tooltip
                  key='fullWidth'
                  title="This property spans the width of the block"
                  sx={{ fontSize: 20 }}
                  arrow
                >
                  <Avatar
                    sx={{
                      width: 30,
                      height: 30,
                      backgroundColor: DOC_FLAG_COLORS.FULL_WIDTH,
                    }}
                  >
                    <FiCode />
                  </Avatar>
                </Tooltip>
              )}
              {fieldInfo.isRequired && (
                <Tooltip
                  key='isRequired'
                  title="This property is required"
                  sx={{ fontSize: 20 }}
                  arrow
                >
                  <Avatar
                    sx={{
                      width: 30,
                      height: 30,
                      backgroundColor: DOC_FLAG_COLORS.REQUIRED,
                    }}
                  >
                    <FiStar />
                  </Avatar>
                </Tooltip>
              )}
              {fieldInfo.isFunctionArgument && (
                <Tooltip
                  key='isFunctionArgument'
                  title="This is an argument to the function"
                  sx={{ fontSize: 20 }}
                  arrow
                >
                  <Avatar
                    sx={{
                      width: 30,
                      height: 30,
                      backgroundColor: DOC_FLAG_COLORS.FUNCTION_ARGUMENT,
                    }}
                  >
                    <FiDownload />
                  </Avatar>
                </Tooltip>
              )}
              {fieldInfo.isFunctionBlockField && (
                <Tooltip
                  key="isFunctionBlockField"
                  title="This is a property of this function block"
                  sx={{ fontSize: 20 }}
                  arrow
                >
                  <Avatar
                    sx={{
                      width: 30,
                      height: 30,
                      backgroundColor: DOC_FLAG_COLORS.FUNCTION_PROPERTY,
                    }}
                  >
                    <FiSquare />
                  </Avatar>
                </Tooltip>
              )}
            </Stack>
          ) : (
            <Typography
              sx={{ fontSize: 14, textAlign: "center", fontStyle: "italic" }}
              color="text.secondary"
              gutterBottom={false}
            >
              {fieldInfo.type}
            </Typography>
          )
        }
      />
      {fieldInfo?.accepts && (
        <CardContent
          sx={{
            bgcolor: "#252525",
            borderRadius: 1,
            padding: 0.5,
            lineHeight: 1.75,
          }}
        >
          {fieldInfo?.accepts?.map((t) => (
            <TypeLink
              key={t}
              label={typeInfo[t]?.name || "Unrecognized"}
              color={getColor(typeInfo[t])}
              onClick={(e) => {
                e.preventDefault();
                if (typeInfo[t]) {
                  handleLinkClick(t);
                }
              }}
            />
          ))}
        </CardContent>
      )}
    </Card>
  );
};

const DOC_FLAG_COLORS = {
  INBOUND_CONNECTION: "#DDA0DD",
  OUTBOUND_CONNECTION: "#8FBC8F",
  IS_LIST: "#6495ED",
  FULL_WIDTH: "#ADD8E6",
  REQUIRED: "#FF6347",
  FUNCTION_ARGUMENT: "#F5DEB3",
  FUNCTION_PROPERTY: "#00CED1",
};

const ConnectionInfo = ({
  side,
  connectionInfo,
  typeInfo,
  handleLinkClick,
}) => {
  return (
    <Card sx={{ padding: 2 }}>
      <Grid container spacing={1}>
        <Grid item xs={4}>
          <Typography
            variant="h5"
            color="text.primary"
            style={{ textTransform: "capitalize" }}
          >
            {side}
          </Typography>
        </Grid>
        <Grid item xs={8}>
          <Typography
            sx={{ fontSize: 14, textAlign: "center", fontStyle: "italic" }}
            color="text.secondary"
            gutterBottom={false}
          >
            Accepts
          </Typography>
        </Grid>
        {connectionInfo.allowed && (
          <>
            <Grid item xs={4}>
              <Tooltip
                title={
                  connectionInfo.direction === CONNECTIONS.OUTBOUND
                    ? "This connection is outbound"
                    : "This connection is inbound"
                }
                sx={{ fontSize: 20 }}
                arrow
              >
                <Avatar
                  sx={{
                    width: 30,
                    height: 30,
                    backgroundColor:
                      connectionInfo.direction === CONNECTIONS.OUTBOUND
                        ? DOC_FLAG_COLORS.OUTBOUND_CONNECTION
                        : DOC_FLAG_COLORS.INBOUND_CONNECTION,
                  }}
                >
                  {connectionInfo.direction === CONNECTIONS.OUTBOUND ? (
                    <FiLogOut />
                  ) : (
                    <FiLogIn />
                  )}
                </Avatar>
              </Tooltip>
            </Grid>
            <Grid
              item
              xs={8}
              sx={{
                bgcolor: "#252525",
                borderRadius: 1,
                padding: 3,
                lineHeight: 1.75,
              }}
            >
              {connectionInfo.allowed?.map((t) => (
                <TypeLink
                  label={typeInfo[t].name}
                  color={getColor(typeInfo[t])}
                  onClick={(e) => {
                    e.preventDefault();
                    if (typeInfo[t]) {
                      handleLinkClick(t);
                    }
                  }}
                />
              ))}
            </Grid>
          </>
        )}
      </Grid>
    </Card>
  );
};