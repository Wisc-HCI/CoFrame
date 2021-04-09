import React, { Fragment } from "react";

import { SettingsModal } from "./SettingsModal";
import { UploadModal } from "./UploadModal";
import { DownloadModal } from "./DownloadModal";
import { OpenModal } from "./OpenModel";
import { SaveModal } from "./SaveModal";


export const Modals = (props) => {

    const { totalWidth } = props;

    return (
        <Fragment>
            
            <DownloadModal
                totalWidth={totalWidth}
            />

            <SaveModal 
                totalWidth={totalWidth}
            />

            <UploadModal
                totalWidth={totalWidth}
            />

            <OpenModal
                totalWidth={totalWidth}
            />

            <SettingsModal
                totalWidth={totalWidth}
            />

        </Fragment>
    );
};