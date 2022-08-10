import React, { useRef, useState } from "react";
// import { TextInput, List } from "grommet";
import { FiRotateCw, FiDownload, FiUpload } from "react-icons/fi";
import useStore from "../stores/Store";
import { saveAs } from "file-saver";
import YAML from "yaml";
import ReactJson from "react-json-view";
import {
  Dialog,
  Tab,
  Tabs,
  Alert,
  AlertTitle,
  Box,
  Stack,
  TextField,
  InputAdornment,
  IconButton,
  Button,
  Typography,
  Card,
  CardContent,
} from "@mui/material";
import { FrameTabBar } from "./FrameTabBar";
import shallow from "zustand/shallow";
import { ScrollRegion } from "./Elements/ScrollRegion";

const DialogContent = () => {
  // const url = useStore((store) => store.url, shallow);
  const setUrl = useStore((store) => store.setUrl, shallow);
  const connection = useStore((store) => store.connection, shallow);
  const connect = useStore((store) => store.connect, shallow);
  const issueSettings = useStore((store) => store.issueSettings, shallow);
  const updateIssueSetting = useStore(
    (store) => store.updateIssueSetting,
    shallow
  );
  const setData = useStore((store) => store.setData, shallow);
  const data = useStore((store) => store.programData, shallow);
  const frameId = useStore((store) => store.frame, shallow);
  const setFrame = useStore((store) => store.setFrame, shallow);

  const [prefix, setPrefix] = useState("ws://");
  const [host, setHost] = useState("localhost:9090");

  const togglePrefix = () => {
    const newPrefix = prefix === "ws://" ? "wss://" : "ws://";
    setPrefix(newPrefix);
    setUrl(newPrefix + host);
  };

  const updateHost = (newHost) => {
    setHost(newHost);
    setUrl(prefix + newHost);
  };

  const fileInputRef = useRef();

  const download = () => {
    const text = JSON.stringify(data);

    let name = "default_program";
    Object.values(data).some((block) => {
      if (block.type === "programType") {
        name = block.name;
        return true;
      } else {
        return false;
      }
    });

    const blob = new Blob([text], { type: "text/plain;charset=utf-8" });
    saveAs(blob, `${name.replace(" ", "_")}.json`);
  };

  const upload = async (event) => {
    const fileUploaded = event.target.files[0];
    if (fileUploaded) {
      const reader = new FileReader();
      reader.onabort = () => {
        /* message.error('Upload Aborted') */
      };
      reader.onerror = () => {
        /* message.error('Upload Error') */
      };
      reader.onload = () => {
        let data = YAML.parse(reader.result);
        if (data) {
          // Do handling
          setData(data);
        }
      };
      reader.readAsText(fileUploaded);
    }
  };

  const handleUploadClick = (_) => {
    fileInputRef.current.click();
  };

  const updateIssue = (value, issue) => {
    const item = {
      id: issue.id,
      name: issue.name,
      frame: issue.frame,
      value: parseFloat(value),
    };
    updateIssueSetting(item);
  };

  const [tab, setTab] = useState("settings");

  const filteredIssueSettings = Object.values(issueSettings).filter(
    (v) => v.frame === frameId
  );

  return (
    <Card sx={{ padding: 0 }}>
      <Box
        sx={{ borderBottom: 1, borderColor: "#444", backgroundColor: "#222" }}
      >
        <Tabs
          value={tab}
          onChange={(_, newValue) => {
            setTab(newValue);
          }}
          centered
        >
          <Tab value="settings" label="Settings" />
          <Tab value="debug" label="Debug" />
        </Tabs>
      </Box>
      <CardContent
        sx={{
          // height: "70vh",
          // width: "50vw",
          flexDirection: "row",
          display: "flex",
          padding: 0,
          margin: 0,
        }}
      >
        <Box
          // background="#252525"

          // margin={{ top: "small" }}
          // round="xsmall"
          sx={{
            overflowY: "scroll",
            maxHeight: "70vh",
            width: "50vw",
            // flex: 1,
            // width:'calc(50vw - 45px)',
            // height:'calc(70vh - 10px)',
          }}
        >
          {tab === "settings" ? (
            <Stack direction="column" spacing={1} sx={{ padding: "5px" }}>
              {/* ROS Connection (Not used currently) */}
              <Alert
                sx={{ marginBottom: "5px" }}
                variant="filled"
                severity={
                  connection === "connected"
                    ? "success"
                    : connection === "connecting"
                    ? "info"
                    : "error"
                }
              >
                <AlertTitle>
                  {connection === "connected"
                    ? "Connected"
                    : connection === "Connecting..."
                    ? "info"
                    : "Disconnected"}
                </AlertTitle>
                {connection === "connected"
                  ? "You are connected to a ROS Server"
                  : connection === "connecting"
                  ? "You are connecting to a ROS Server"
                  : "You are not connected to a ROS Server"}
              </Alert>

              <TextField
                // placeholder="e.g. ws://localhost:9090"
                label="URL"
                value={host}
                onChange={(e) => updateHost(e.target.value)}
                fullWidth
                InputProps={{
                  className: "nodrag",
                  style: { paddingRight: 6 },
                  startAdornment: (
                    <InputAdornment position="start">
                      <Button onClick={togglePrefix}>{prefix}</Button>
                    </InputAdornment>
                  ),
                  endAdornment: (
                    <InputAdornment position="end">
                      <IconButton onClick={connect}>
                        <FiRotateCw style={{ height: 14, width: 14 }} />
                      </IconButton>
                    </InputAdornment>
                  ),
                }}
              />

              {/* Upload/Download */}
              <input
                type="file"
                ref={fileInputRef}
                onChange={upload}
                style={{ display: "none" }}
              />
              <Stack direction="row" spacing={1} justifyContent="between">
                <Button
                  variant="outlined"
                  icon={<FiUpload />}
                  style={{ flex: 1 }}
                  onClick={handleUploadClick}
                >
                  Upload
                </Button>
                <Button
                  variant="outlined"
                  icon={<FiDownload />}
                  style={{ flex: 1 }}
                  label="Download"
                  onClick={download}
                >
                  Download
                </Button>
              </Stack>

              {/* Frames Selector */}
              <Stack
                direction="column"
                alignItems="center"
                alignContent="center"
                sx={{ height: 60 }}
              >
                <FrameTabBar
                  active={frameId}
                  onChange={setFrame}
                  backgroundColor={"inherit"}
                />
              </Stack>
              {/* Expert Settings */}
              <ScrollRegion
                vertical
                height={"calc(70vh - 280px)"}
                width="100%"
                style={{
                  backgroundColor: "#222222",
                  borderRadius: 4,
                  paddingTop: 4,
                }}
              >
                <Stack
                  // sx={{ padding: '5px' }}
                  spacing={2}
                  direction="column"
                  sx={{
                    // width: "calc(50vw - 25px)",
                    padding: "5px",
                  }}
                >
                  {filteredIssueSettings.length > 0 ? (
                    filteredIssueSettings.map((entry) => (
                      <Box
                        key={entry.name.concat("input")}
                        sx={{
                          display: "flex",
                          // width: "calc(50vw - 40px)",
                          padding: "4px",
                        }}
                      >
                        {!entry.max && (
                          <TextField
                            fullWidth
                            label={entry.name}
                            type="number"
                            min={entry.min}
                            value={entry.value}
                            onChange={(e) => updateIssue(e.target.value, entry)}
                          />
                        )}
                        {entry.max && (
                          <TextField
                            fullWidth
                            label={entry.name}
                            type="number"
                            min={entry.min}
                            max={entry.max}
                            value={entry.value}
                            onChange={(e) => updateIssue(e.target.value, entry)}
                          />
                        )}
                      </Box>
                    ))
                  ) : (
                    <Typography sx={{ width: "100%", textAlign: "center" }}>
                      No Settings
                    </Typography>
                  )}
                </Stack>
              </ScrollRegion>
            </Stack>
          ) : tab === "debug" ? (
            <ReactJson src={data} collapsed={1} theme="tomorrow" />
          ) : null}
        </Box>
      </CardContent>
    </Card>
  );
};

export const SettingsModal = () => {
  const closeModal = useStore((store) => store.closeModal, shallow);
  const activeModal = useStore((store) => store.activeModal, shallow);

  return (
    <Dialog open={Boolean(activeModal)} onBackdropClick={closeModal}>
      {activeModal && <DialogContent />}
    </Dialog>
  );
};
