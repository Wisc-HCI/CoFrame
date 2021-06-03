import React, { Fragment } from "react";

import { LocationDetail } from "./LocationDetail";
import {MachineDetail} from "./MachineDetail";
import {WaypointDetail} from "./WaypointDetail";
import {RegionDetail} from "./RegionDetail.jsx";
import {ThingDetail} from "./ThingDetail.jsx";


export const Detail = (props) => {

    return (
        <Fragment>

            <LocationDetail/>
            <MachineDetail/>
            <WaypointDetail/>
            <ThingDetail/>
            {/* <MachineDetail/>
            <WaypointDetail/>
            <RegionDetail/> */}

        </Fragment>
    );
};
