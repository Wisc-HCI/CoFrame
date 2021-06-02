import React, { Fragment } from "react";

import { LocationDetail } from "./LocationDetail";
import {MachineDetail} from "./MachineDetail";
import {WaypointDetail} from "./WaypointDetail";


export const Detail = (props) => {

    return (
        <Fragment>

            <LocationDetail/>
            <MachineDetail/>
            <WaypointDetail/>

        </Fragment>
    );
};
