import React, { Fragment } from "react";

import { LocationDetail } from "./LocationDetail";
import {MachineDetail} from "./MachineDetail"


export const Detail = (props) => {

    return (
        <Fragment>

            <LocationDetail/>
            <MachineDetail/>

        </Fragment>
    );
};
