import React from 'react';


/*
ThemeContext : 
{
    theme: <theme obj>
    frameStyles: <frame styling obj>
    toggleTheme: <fnt() "switch from light to dark">
    themeName: <string "light or dark">
}
*/
export const ThemeContext = React.createContext(null);

/*
FrameContext : 
{
    frame: <string "frame name">
    changeFrame: <fnt(string) "updates current active frame"> 
}
*/
export const FrameContext = React.createContext(null);

/*
ControlsContext :
{
    inSetup: <bool true is in setup, false is in editor>
    checklistItem: <string "uuid of checklist item">
    setupItem: <string "uuid of setup item">
    changeSetupState: <fnt(bool) sets inSetup>
    changeChecklistItem: <fnt(string) sets checklistItem>
    changeSetupItem: <fnt(string) sets setupItem>
}
*/
export const ControlsContext = React.createContext(null);

/*
ApplicationContext : 
{
    service: <appService obj>
    ...appService.state
}
*/
export const ApplicationContext = React.createContext(null);

/*
RosContext : 
{
    service: <rosService obj>
    ...rosService.state
}
*/
export const RosContext = React.createContext(null);

/*
EvDScriptContext : 
{
    service: <evdService obj>
    ...evdService.state
}
*/
export const EvDScriptContext = React.createContext(null);

/*
PendingContext :
{
    service: <pendingService obj>
    ...pendingService.state
}
*/
export const PendingContext = React.createContext(null);

/*
UnityContext :
{
    service: <unityService obj>
    ...unityService.state
}
*/
export const UnityContext = React.createContext(null);

/*
ModalContext : 
{
    closeModal: <fnt(name)>
    openModal: <fnt(name)>
}
*/
export const ModalContext = React.createContext(null);