import { Node } from '../node';
import { Trace } from './trace';

export class Trajectory extends Node {

    /*
    * Class Constants
    */

    static TYPES = ['joint', 'linear', 'planner'];

    /*
    * Data structure methods
    */

    static typeString() {
        return 'trajectory.';
    }

    static fullTypeString() {
        return Node.fullTypeString() + Trajectory.typeString();
    }

    constructor(startLocUuid=null, endLocUuid=null, waypointUuids=[], trace=null, 
                moveType='joint', velocity=0, acceleration=0, 
                type='', name='', uuid=null, parent=null, appendType=true) 
    {
        super(
            (appendType) ? 'trajectory.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );

        this._startLocationUuid = null;
        this._endLocationUuid = null;
        this._waypointUuids = null;
        this._velocity = null;
        this._acceleration = null;
        this._trace = null;
        this._moveType = null;

        this.startLocationUuid = startLocUuid;
        this.endLocationUuid = endLocUuid;
        this.waypointUuids = waypointUuids;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.trace = trace;
        this.moveType = moveType;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            start_location_uuid: this.startLocationUuid,
            end_location_uuid: this.endLocationUuid,
            waypoint_uuids: this.waypointUuids,
            velocity: this.velocity,
            acceleration: this.acceleration,
            trace: (this.trace !== null) ? this.trace.toDict() : null,
            move_type: this.moveType
        };
        return msg;
    }

    static fromDict(dct) {
        return new Trajectory(
            dct.start_location_uuid,
            dct.end_location_uuid,
            dct.waypoint_uuids,
            (dct.trace !== null) ? Trace.fromDict(dct.trace) : null,
            dct.move_type,
            dct.velocity,
            dct.acceleration,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }

    static BlocklyToolbox() {
        return { type: 'trajectory' };
    }

    static BlocklyBlock() {
        return { key: 'trajectory', data: {
            init: function() {
                this.appendDummyInput()
                    .appendField("Trajectory");
                this.appendValueInput("start-location")
                    .setCheck("location")
                    .appendField("Start Location");
                this.appendStatementInput("waypoints")
                    .setCheck("waypoint")
                    .appendField("Waypoints");
                this.appendValueInput("end-location")
                    .setCheck("location")
                    .appendField("End Location");
                this.setOutput(true, null);
                this.setColour(330);
                this.setTooltip("trajectory");
                this.setHelpUrl("trajectory");
            }
        }};
    }

    /*
    * Data accessor/modifier methods
    */

    get startLocationUuid() {
        return this._startLocationUuid;
    }

    set startLocationUuid(value) {
        if (this._startLocationUuid !== value) {
            this._startLocationUuid = value;
            this.trace = null;
            this.updatedAttribute('start_location_uuid','set');
        }
    }

    get endLocationUuid() {
        return this._endLocationUuid;
    }

    set endLocationUuid(value) {
        if (this._endLocationUuid !== value) {
            this._endLocationUuid = value;
            this.trace = null;
            this.updatedAttribute('end_location_uuid','set');
        }
    }

    get waypointUuids() {
        return this._waypointUuids;
    }

    set waypointUuids(value) {
        if (this._waypointUuids !== value) {
            if (value === null) {
                throw new Error('Waypoints must be a list not None');
            }

            this._waypointUuids = value;
            this.trace = null;
            this.updatedAttribute('waypoint_uuids','set');
        }
    }

    get trace() {
        return this._trace;
    }

    set trace(value) {
        if (this._trace !== value) {
            if (this._trace !== null) {
                this._trace.removeFromCache();
            }

            this._trace = value;
            if (this._trace !== null) {
                this._trace.parent = this;
            }

            this.updatedAttribute('trace','set');
        }
    }

    get velocity() {
        return this._velocity;
    }

    set velocity(value) {
        if (this._velocity !== value) {
            this._velocity = value;
            this.trace = null;
            this.updatedAttribute('velocity','set');
        }
    }

    get acceleration() {
        return this._acceleration;
    }

    set acceleration(value) {
        if (this._acceleration !== value) {
            this._acceleration = value;
            this.trace = null;
            this.updatedAttribute('acceleration','set');
        }
    }

    get moveType() {
        return this._moveType;
    }

    set moveType(value) {
        if (this._moveType !== value) {
            if (! (value in Trajectory.TYPES)) {
                throw new Error('Invalid moveType provided');
            }

            this._moveType = value;
            this.trace = null;
            this.updatedAttribute('move_type','set');
        }
    }

    addWaypointUuid(uuid) {
        this._waypointUuids.push(uuid);
        this.trace = null;
        this.updatedAttribute('waypoint_uuids','add',uuid);
    }

    insertWaypointUuid(uuid, idx) {
        this._waypointUuids.splice(idx, 0, uuid);
        this.trace = null;
        this.updatedAttribute('waypoint_uuids','add',uuid);
    }

    reorderWaypointUuids(uuid, shift) {
        const idx = this.findWaypointIndex(uuid);

        if (idx !== null) {
            let shiftedIdx = idx + shift;
            if (shiftedIdx < 0 || shiftedIdx >= this._waypointUuids.length) {
                throw new Error('Index out of bounds');
            }

            const copy = this._waypointUuids.splice(idx, 1);
            this._waypointUuids.splice(shiftedIdx, 0, copy);
            this.updatedAttribute('waypoint_uuids', 'reorder');
        }
    }

    deleteWaypointUuid(uuid) {
        const idx = this.findWaypointIndex(uuid);

        if (idx !== null) {
            this._waypointUuids.splice(idx, 1);
            this.trace = null;
            this.updatedAttribute('waypoint_uuids','delete',uuid);
        }
    }

    findWaypointIndex(uuid) {
        let idx = null;
        for (let i=0; i<this._waypointUuids.length; i++) {
            if (this._waypointUuids[i].uuid === uuid) {
                idx = i;
                break;
            }
        }
        return idx;
    }

    set(dct) {

        if ('start_location_uuid' in dct) {
            this.startLocationUuid = dct.start_location_uuid;
        }

        if ('end_location_uuid' in dct) {
            this.endLocationUuid = dct.end_location_uuid;
        }

        if ('waypoint_uuids' in dct) {
            this.waypointUuids = dct.waypoint_uuids;
        }

        if ('velocity' in dct) {
            this.velocity = dct.velocity;
        }

        if ('acceleration' in dct) {
            this.acceleration = dct.acceleration;
        }

        if ('move_type' in dct) {
            this.moveType = dct.move_type;
        }

        if ('trace' in dct) {
            this.trace = (dct.trace !== null) ? Trace.fromDict(dct.trace) : null;
        }

        super.set(dct);
    }

    /*
    * Cache methods
    */

    removeFromCache() {
        if (this.trace !== null) {
            this.trace.removeFromCache();
        }

        super.removeFromCache();
    }

    addToCache() {
        if (this.trace !== null) {
            this.trace.addToCache();
        }

        super.addToCache();
    }

    /*
    * Children methods
    */

    deleteChild(uuid) {
        let success = false;

        if (this.trace !== null && this.trace.uuid === uuid) {
            this.trace = null;
            success = true;
        }

        return success;
    }

    /*
    * Update methods
    */

    lateConstructUpdate() {
        if (this.trace !== null) {
            this.trace.lateConstructUpdate();
        }
        
        super.lateConstructUpdate();
    }

    deepUpdate() {
        if (this.trace !== null) {
            this.trace.deepUpdate();
        }

        super.deepUpdate();

        this.updatedAttribute('start_location_uuid','update');
        this.updatedAttribute('end_location_uuid','update');
        this.updatedAttribute('waypoint_uuids','update');
        this.updatedAttribute('velocity','update');
        this.updatedAttribute('acceleration','update');
        this.updatedAttribute('move_type','update');
        this.updatedAttribute('trace','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('start_location_uuid','update');
        this.updatedAttribute('end_location_uuid','update');
        this.updatedAttribute('waypoint_uuids','update');
        this.updatedAttribute('velocity','update');
        this.updatedAttribute('acceleration','update');
        this.updatedAttribute('move_type','update');
        this.updatedAttribute('trace','update');
    }
}