import { Context } from '../context';
import { ReachSphere } from './reach_sphere';
import { CollisionMesh } from './collision_mesh';
import { OccupancyZone } from './occupancy_zone';
import { PinchPoint } from './pinch_point';
import { Location } from '../data/location';
import { Machine } from '../data/machine';
import { Thing } from '../data/thing';
import { Waypoint } from '../data/waypoint';
import { Trajectory } from '../data/trajectory';
import { EnvironmentNodeParser } from '.';

export class Environment extends Context {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'environment.';
    }

    static fullTypeString() {
        return Context.fullTypeString() + Environment.typeString();
    }

    constructor(reachSphere=null, pinchPoints=[], collisionMeshes=[], occupancyZones=[], locations=[], machines=[], things=[], waypoints=[], trajectories=[], name='', type='', uuid=null, parent=null, appendType=true) {
        this._reachSphere = null;
        this._pinchPoints = null;
        this._collisionMeshes = null;
        this._occupancyZones = null;

        super(
            locations= locations,
            machines= machines,
            things= things,
            waypoints= waypoints,
            trajectories= trajectories,
            type= (appendType) ? 'environement.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.reachSphere = reachSphere;
        this.pinchPoints = pinchPoints;
        this.collisionMeshes = collisionMeshes;
        this.occupancyZones = occupancyZones;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            reach_sphere: this.ReachSphere.toDict(),
            pinch_points: this.pinchPoints.map(p => p.toDict()),
            collision_meshes: this.collisionMeshes.map(c => c.toDict()),
            occupancy_zones: this.occupancyZones.map(o => o.toDict())
        };
        return msg;
    }

    static fromDict(dct) {
        return new Environment(
            reachSphere= ReachSphere.fromDict(dct.reach_sphere),
            pinchPoints= dct.pinch_points.map(p => PinchPoint.fromDict(p)),
            collisionMeshes= dct.collision_meshes.map(c => CollisionMesh.fromDict(c)),
            occupancyZones= dct.occupancy_zones.map(o => OccupancyZone.fromDict(o)),
            locations= dct.locations.map(l => Location.fromDict(l)),
            machines= dct.machines.map(m => Machine.fromDict(m)),
            things= dct.things.map(t => Thing.fromDict(t)),
            waypoints= dct.waypoints.map(w => Waypoint.fromDict(w)),
            trajectories= dct.trajectories.map(t => Trajectory.fromDict(t)),
            type= dct.type,
            uuid= dct.uuid,
            name= dct.name,
            appendType= false
        );
    }

    /*
    * Data accessor/modifier methods
    */

    get reachSphere() {
        return this._reachSphere;
    }

    set reachSphere(value) {
        if (value === null) {
            throw new Error('reach sphere cannot be null');
        }

        if (this._reachSphere !== value) {
            if (this._reachSphere !== null) {
                this._reachSphere.removeFromCache();
            }

            this._reachSphere = value;
            this._reachSphere.parent = this;
            this.updatedAttribute('reach_sphere','set',this._reachSphere.uuid);
        }
    }

    get pinchPoints() {
        return this._pinchPoints;
    }

    set pinchPoints(value) {
        if (value === null) {
            throw new Error('pinch point list cannot be null');
        }

        if (this._pinchPoints !== value) {
            if (this._pinchPoints !== null) {
                for (const p in this._pinchPoints) {
                    p.removeFromCache();
                }
            }

            this._pinchPoints = value;
            for (const p in this._pinchPoints) {
                p.parent = this;
            }

            this.updatedAttribute('pinch_points','set');
        }
    }

    get collisionMeshes() {
        return this._collisionMeshes;
    }

    set collisionMeshes(value) {
        if (value === null) {
            throw new Error('collision mesh list cannot be null');
        }

        if (this._collisionMeshes !== value) {
            if (this._collisionMeshes !== null) {
                for (const c in this._collisionMeshes) {
                    c.removeFromCache();
                }
            }

            this._collisionMeshes = value;
            for (const c in this._collisionMeshes) {
                c.parent = this;
            }

            this.updatedAttribute('collision_meshes','set');
        } 
    }

    get occupancyZones() {
        return this._occupancyZones;
    }

    set occupancyZones(value) {
        if (value === null) {
            throw new Error('occupancy zone list cannot be null');
        }

        if (this._occupancyZones !== value) {
            if (this._occupancyZones !== null) {
                for (const o in this._occupancyZones) {
                    o.removeFromCache();
                }
            }

            this._occupancyZones = value;
            for (const o in this._occupancyZones) {
                o.parent = this;
            }

            this.updatedAttribute('occupancy_zones','set');
        }
    }

    set(dct) {

        if ('reach_sphere' in dct) {
            this.reachSphere = ReachSphere.fromDict(dct.reach_sphere);
        }

        if ('pinch_points' in dct) {
            this.pinchPoints = dct.pinch_points.map(p => PinchPoint.fromDict(p));
        }

        if ('collision_meshes' in dct) {
            this.CollisionMesh = dct.collision_meshes.map(c => CollisionMesh.fromDict(c));
        }

        if ('occupancy_zones' in dct) {
            this.occupancyZones = dct.occupancy_zones.map(o => OccupancyZone.fromDict(o));
        }

        super.set(dct);
    }

    /*
    * Cache methods
    */

    removeFromCache() {
        this.reachSphere.removeFromCache();
        this.pinchPoints.forEach(p => p.removeFromCache());
        this.collisionMeshes.forEach(c => c.removeFromCache());
        this.occupancyZones.forEach(o => o.removeFromCache());
        super.removeFromCache();
    }

    addToCache() {
        this.reachSphere.addToCache();
        this.pinchPoints.forEach(p => p.addToCache());
        this.collisionMeshes.forEach(c => c.addToCache());
        this.occupancyZones.forEach(o => o.addToCache());
        super.addToCache();
    }

    /*
    * Update methods
    */

    lateConstructUpdate() {
        this.reachSphere.lateConstructUpdate();
        this.pinchPoints.forEach(p => p.lateConstructUpdate());
        this.collisionMeshes.forEach(c => c.lateConstructUpdate());
        this.occupancyZones.forEach(o => o.lateConstructUpdate());

        super.lateConstructUpdate();
    }

    deepUpdate() {
        this.reachSphere.deepUpdate();
        this.pinchPoints.forEach(p => p.deepUpdate());
        this.collisionMeshes.forEach(c => c.deepUpdate());
        this.occupancyZones.forEach(o => o.deepUpdate());

        super.deepUpdate();

        this.updatedAttribute('reach_sphere','update');
        this.updatedAttribute('pinch_points','update');
        this.updatedAttribute('collision_meshes','update');
        this.updatedAttribute('occupancy_zones','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('reach_sphere','update');
        this.updatedAttribute('pinch_points','update');
        this.updatedAttribute('collision_meshes','update');
        this.updatedAttribute('occupancy_zones','update');
    }
}