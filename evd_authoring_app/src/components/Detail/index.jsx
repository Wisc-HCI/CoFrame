import React, { Fragment } from "react";

import { LocationDetail } from "./LocationDetail";
import {MachineDetail} from "./MachineDetail";
import {WaypointDetail} from "./WaypointDetail";
import {RegionDetail} from "./RegionDetail.jsx";


export const Detail = (props) => {

    return (
        <Fragment>

            <LocationDetail/>
            {/* <MachineDetail/>
            <WaypointDetail/>
            <RegionDetail/> */}

        </Fragment>
    );
};
