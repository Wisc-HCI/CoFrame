import React, { Fragment } from "react";

import { SettingsModal } from "./SettingsModal";
import { SyncModal } from "./SyncModal";


export const Modals = (props) => {

    return (
        <Fragment>
            
            <SyncModal/>

            <SettingsModal/>

        </Fragment>
    );
};