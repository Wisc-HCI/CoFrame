import { Node } from './node';
import { Thing } from './data/thing';
import { Machine } from './data/machine';
import { Waypoint } from './data/waypoint';
import { Location } from './data/location';
import { Trajectory } from './data/trajectory';


export class Context {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'context.';
    }

    static fullTypeString() {
        return Node.fullTypeString() + Context.typeString();
    }

    constructor(locations=[], machines=[], things=[], waypoints=[], trajectories=[],
                type='', name='', uuid=null, parent=null, appendType=null) 
    {
        this._locations = {};
        this._machines = {};
        this._things = {};
        this._waypoints = {};
        this._trajectories = {};
        
        super(
            type= (appendType) ? 'context.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.locations = locations;
        this.machines = machines;
        this.things = things;
        this.waypoints = waypoints;
        this.trajectories = trajectories;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            locations: this.locations.map(l => l.toDict()),
            machines: this.machines.map(m => m.toDict()),
            things: this.things.map(t => t.toDict()),
            waypoints: this.waypoints.map(w => w.toDict()),
            trajectories: this.trajectories.map(t => t.toDict())
        };
        return msg;
    }

    static fromDict(dct) {
        return Context(
            name= dct.name,
            uuid= dct.uuid,
            type= dct.type,
            appendType: false,
            locations= dct.locations.map(l => Location.fromDict(l)),
            machines= dct.machines.map(m => Machine.fromDict(m)),
            things= dct.things.map(t => Thing.fromDict(t)),
            waypoints= dct.waypoints.map(w => Waypoint.fromDict(w)),
            trajectories= dct.trajectories.map(t => Trajectory.fromDict(t))
        );
    }

    /*
    * Data accessor/modifier methods
    */

    get context() {
        return this;
    }

    get locations() {
        return Object.values(this._locations);
    }

    set locations(value) {
        
        Object.values(this._locations).forEach(element => {
            element.removeFromCache();
        });
        this._locations = {};

        value.forEach(element => {
            this._locations[element.uuid] = element;
            element.parent = this;
        });

        this.updatedAttribute('locations','set');
    }

    getLocation(uuid) {
        if (uuid in this._locations) {
            return this._locations[uuid];
        } else {
            return null;
        }
    }

    addLocation(location) {
        location.parent = this;
        this._locations[location.uuid] = location;
        this.updatedAttribute('locations','add',location.uuid);
    }

    deleteLocation(uuid) {
        if (uuid in this._locations) {
            this._locations[uuid].removeFromCache();
            delete this._locations[uuid];
            this.updatedAttribute('locations','delete',uuid);
        }
    }

    get machines() {
        return Object.values(this._machines);
    }

    set machines(value) {

        Object.values(this._machines).forEach(element => {
            element.removeFromCache();
        });
        this._machines = {};

        value.forEach(element => {
            this._machines[element.uuid] = element;
            element.parent = this;
        });

        this.updatedAttribute('machines','set');
    }

    getMachine(uuid) {
        if (uuid in this._machines) {
            return this._machines[uuid];
        } else {
            return null;
        }
    }

    addMachine(machine) {
        machine.parent = this;
        this._machines[machine.uuid] = machine;
        this.updatedAttribute('machines','add',machine.uuid);
    }

    deleteMachine(uuid) {
        if (uuid in this._machines) {
            this._machines[uuid].removeFromCache();
            delete this._machines[uuid];
            this.updatedAttribute('machines','delete',uuid);
        }
    }

    get things() {
        return Object.values(this._things);
    }

    set things(value) {

        Object.values(this._things).forEach(element => {
            element.removeFromCache();
        });
        this._things = {};

        value.forEach(element => {
            this._things[element.uuid] = element;
            element.parent = this;
        });

        this.updatedAttribute('things','set');
    }

    getThing(uuid) {
        if (uuid in this._things) {
            return this._things[uuid];
        } else {
            return null;
        }
    }

    addThing(thing) {
        thing.parent = this;
        this._things[thing.uuid] = thing;
        this.updatedAttribute('thigns','add',thing.uuid);
    }

    deleteThing(uuid) {
        if (uuid in this._things) {
            this._things[uuid].removeFromCache();
            delete this._things[uuid];
            this.updatedAttribute('things','delete',uuid);
        }
    }

    get waypoints() {
        return Object.values(this._waypoints);
    }

    set waypoints(value) {

        Object.values(this._waypoints).forEach(element => {
            element.removeFromCache();
        });

        value.forEach(element => {
            this._waypoints[element.uuid] = element;
            element.parent = this;
        });

        this.updatedAttribute('waypoints','set');
    }

    getWaypoint(uuid) {
        if (uuid in this._waypoints) {
            return this._waypoints[uuid];
        } else {
            return null;
        }
    }

    addWaypoint(waypoint) {
        waypoint.parent = this;
        this._waypoints[waypoint.uuid] = waypoint;
        this.updatedAttribute('waypoints','add',waypoint.uuid);
    }

    deleteWaypoint(uuid) {
        if (uuid in this._waypoints) {
            this._waypoints[uuid].removeFromCache();
            delete this._waypoints[uuid];
            this.updatedAttribute('waypoints','delete',uuid);
        }
    }

    get trajectories() {
        return Object.values(this._trajectories);
    }

    set trajectories(value) {

        Object.values(this._trajectories).forEach(element => {
            element.removeFromCache();
        });

        value.forEach(element => {
            this._trajectories[element.uuid] = element;
            element.parent = this;
        });

        this.updatedAttribute('trajectories','set');
    }

    getTrajectory(uuid) {
        if (uuid in this._trajectories) {
            return this._trajectories[uuid];
        } else {
            return null;
        }
    }

    addTrajectory(trajectory) {
        trajectory.parent = this;
        this._trajectories[trajectory.uuid] = trajectory;
        this.updatedAttribute('trajectories','add',trajectory.uuid);
    }

    deleteTrajectory(uuid) {
        if (uuid in this._trajectories) {
            this._trajectories[uuid].removeFromCache();
            delete this._trajectories[uuid];
            this.updatedAttribute('trajectories','delete',uuid);
        }
    }

    set(dct) {

        if ('locations' in dct) {
            this.locations = dct.locations.map(l => this.locations.fromDict(l));
        }

        if ('machines' in dct) {
            this.machines = dct.machines.map(m => Machine.fromDict(m));
        }

        if ('things' in dct) {
            this.things = dct.things.map(t => Thing.fromDict(t));
        }

        if ('waypoints' in dct) {
            this.waypoints = dct.waypoints.map(w => Waypoint.fromDict(w));
        }

        if ('trajectories' in dct) {
            this.trajectories = dct.trajectories.map(t => Trajectory.fromDict(t));
        }

        super.set(dct);
    }

    /*
    * Cache Methods
    */

    removeFromCache() {
        this.machines.forEach(element => element.removeFromCache());
        this.locations.forEach(element => element.removeFromCache());
        this.things.forEach(element => element.removeFromCache());
        this.waypoints.forEach(element => element.removeFromCache());
        this.trajectories.forEach(element => element.removeFromCache());

        super.removeFromCache();
    }

    addToCache() {
        this.machines.forEach(element => element.addToCache());
        this.locations.forEach(element => element.addToCache());
        this.things.forEach(element => element.addToCache());
        this.waypoints.forEach(element => element.addToCache());
        this.trajectories.forEach(element => element.addToCache());

        super.addToCache();
    }

    /*
    * Children methods
    */

    deleteChild(uuid) {

        let success = true;

        if (uuid in this._locations) {
            this.deleteLocation(uuid);
        } else if (uuid in this._machines) {
            this.deleteMachine(uuid);
        } else if (uuid in this._things) {
            this.deleteThing(uuid);
        } else if (uuid in this._waypoints) {
            this.deleteWaypoint(uuid);
        } else if (uuid in this._trajectories) {
            this.deleteTrajectory(uuid);
        } else {
            success = false;
        }

        return success;
    }

    /*
    * Update methods
    */

    lateConstructUpdate() {
        this.machines.forEach(element => element.lateConstructUpdate());
        this.locations.forEach(element => element.lateConstructUpdate());
        this.things.forEach(element => element.lateConstructUpdate());
        this.waypoints.forEach(element => element.lateConstructUpdate());
        this.trajectories.forEach(element => element.lateConstructUpdate());

        super.lateConstructUpdate();
    }

    deepUpdate() {
        this.machines.forEach(element => element.deepUpdate());
        this.locations.forEach(element => element.deepUpdate());
        this.things.forEach(element => element.deepUpdate());
        this.waypoints.forEach(element => element.deepUpdate());
        this.trajectories.forEach(element => element.deepUpdate());

        super.deepUpdate();

        this.updatedAttribute('machines','update');
        this.updatedAttribute('locations','update');
        this.updatedAttribute('things','update');
        this.updatedAttribute('waypoints','update');
        this.updatedAttribute('trajectories','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('machines','update');
        this.updatedAttribute('locations','update');
        this.updatedAttribute('things','update');
        this.updatedAttribute('waypoints','update');
        this.updatedAttribute('trajectories','update');
    }
}