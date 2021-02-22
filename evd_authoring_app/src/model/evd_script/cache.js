
import { Trajectory } from './data/trajectory';
import { Location } from './data/Location';
import { Waypoint } from './data/waypoint';
import { Thing } from './data/thing';
import { Trace } from './data/trace';
import { Machine } from './data/machine';

import { Environment } from './environment/environment';
import { Program } from './program/program';
import { Primitive } from './program/primitive'; 
import { Context } from './context';


// Cache is global so that we can keep a UUID list for NodeParser
cacheObj = null;
export const getEvdCacheObject = () => {
    if (cacheObj == null || cacheObj == undefined) {
        cacheObj = Cache();
    }
    return cacheObj;
}

export class Cache {

    constructor() {
        this.data = {};
        this.trajectories = {};
        this.locations = {};
        this.waypoints = {};
        this.things = {};
        this.traces = {};
        this.machines = {};
        this.environments = {};
        this.programs = {};
        this.primitives = {};
        this.contexts = {};
    }

    add(uuid, node) {
        this.data[uuid] = node;

        if (node instanceof Trajectory) {
            this.trajectories[uuid] = node;
        }

        if (node instanceof Location) {
            this.locations[uuid] = node;
        }

        if (node instanceof Waypoint) {
            this.waypoints[uuid] = node;
        }

        if (node instanceof Thing) {
            this.things[uuid] = node;
        }

        if (node instanceof Trace) {
            this.traces[uuid] = node;
        }

        if (node instanceof Machine) {
            this.machines[uuid] = node;
        }

        if (node instanceof Program) {
            this.programs[uuid] = node;
        }

        if (node instanceof Environment) {
            this.environments[uuid] = node;
        }

        if (node instanceof Primitive) {
            this.primitives[uuid] = node;
        }

        if (node instanceof Context) {
            this.contexts[uuid] = node;
        }
    }

    remove(uuid) {
        node = this.data[uuid];
        delete this.data[uuid];

        if (node instanceof Trajectory) {
            delete this.trajectories[uuid];
        }

        if (node instanceof Location) {
            delete this.locations[uuid];
        }

        if (node instanceof Waypoint) {
            delete this.waypoints[uuid];
        }

        if (node instanceof Thing) {
            delete this.things[uuid];
        }

        if (node instanceof Trace) {
            delete this.traces[uuid];
        }

        if (node instanceof Machine) {
            delete this.machines[uuid];
        }

        if (node instanceof Program) {
            delete this.program[uuid];
        }

        if (node instanceof Environment) {
            delete this.environments[uuid];
        }

        if (node instanceof Primitive) {
            delete this.primitives[uuid];
        }

        if (node instanceof Context) {
            delete this.contexts[uuid];
        }
    }

    clear() {
        this.data = {};
        this.locations = {};
        this.waypoints = {};
        this.trajectories = {};
        this.things = {};
        this.traces = {};
        this.machines = {};
        this.programs = {};
        this.environments = {};
        this.primitives = {};
        this.contexts = {};
    }

    get(uuid, hint=null) {

        let retVal = undefined;

        if (hint === 'trajectory' && uuid in this.trajectories) {
            retVal = this.trajectories[uuid];
        } else if (hint === 'location' && uuid in this.locations) {
            retVal = this.locations[uuid];
        } else if (hint === 'waypoint' && uuid in this.waypoints) {
            retVal = this.waypoints[uuid];
        } else if (hint === 'thing' && uuid in this.things) {
            retVal = this.things[uuid];
        } else if (hint === 'trace' && uuid in this.traces) {
            retVal = this.traces[uuid];
        } else if (hint === 'machine' && uuid in this.machines) {
            retVal = this.machines[uuid];
        } else if (hint === 'program' && uuid in this.programs) {
            retVal = this.programs[uuid];
        } else if (hint === 'environment' && uuid in this.environments) {
            retVal = this.environments[uuid];
        } else if (hint === 'primitive' && uuid in this.primitives) {
            retVal = this.primitives[uuid];
        } else if (hint === 'context' && uuid in this.contexts) {
            retVal = this.contexts[uuid];
        } else {
            retVal = this.data[uuid];
        }

        return retVal;
    }

    set(uuid, dct, hint=null) {
        this.get(uuid,hint).set(dct);
    }

    utilityCacheStats() {
        log = {
            data: {},
            num_trajectories: 0,
            num_locations: 0,
            num_waypoints: 0,
            num_things: 0,
            num_traces: 0,
            num_machines: 0,
            num_primitives: 0,
            num_contexts: 0,
        }

        Object.values(this.data).forEach((node) => {
            if (! node.constructor.name in log.data) {
                log.data[node.constructor.name] = 0;
            }
            log.data[node.constructor.name] += 1;
        });
         
        log.num_trajectories = Object.keys(this.trajectories).length;
        log.num_locations = Object.keys(this.locations).length;
        log.num_waypoints = Object.keys(this.waypoints).length;
        log.num_things = Object.keys(this.things).length;
        log.num_traces = Object.keys(this.traces).length;
        log.num_machines = Object.keys(this.machines).length;
        log.num_primitives = Object.keys(this.primitives).length;
        log.num_contexts = Object.keys(this.contexts).length;
    }

}

