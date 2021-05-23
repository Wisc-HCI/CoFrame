import React, { Fragment } from "react";

import { SettingsModal } from "./SettingsModal";
import { UploadModal } from "./UploadModal";
import { DownloadModal } from "./DownloadModal";
import { OpenModal } from "./OpenModel";
import { SaveModal } from "./SaveModal";


export const Modals = (props) => {

    return (
        <Fragment>
            
            <DownloadModal/>

            <SaveModal/>

            <UploadModal/>

            <OpenModal/>

            <SettingsModal/>

        </Fragment>
    );
};