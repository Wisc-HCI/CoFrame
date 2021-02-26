import { Primitive } from '../primitive';
import { Trajectory } from '../../data/trajectory';
import { Context } from '../../context';


export class MoveTrajectory extends Primitive {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'move-trajectory.';
    }

    static fullTypeString() {
        return Primitive.fullTypeString() + MoveTrajectory.typeString();
    }

    constructor(startLocUuid=null, endLocUuid=null, trajectory=null, trajectoryUuid=null, type='', name='', uuid=null, parent=null, appendType=true) {
        this._contextPatch = null;
        this._startLocationUuid = null;
        this._endLocationUuid = null;
        this._trajectoryUuid = null;

        super(
            type= (appendType) ? 'move-trajectory.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.startLocationUuid = startLocUuid;
        this.endLocationUuid = endLocUuid;

        if (trajectory !== null && trajectoryUuid !== null) {
            throw new Error('Cannot supply both a default trajectory and a trajectory id already in context');
        } else if (trajectory !== null) {
            this.trajectory = trajectory;
        } else if (trajectoryUuid !== null) {
            this.trajectoryUuid = trajectoryUuid;
        } else {
            this.trajectory = new Trajectory(this.startLocationUuid,this.endLocationUuid);
        }
    }

    toDict() {
        const tmp = this.context; // force a context update before serialization

        let msg = {
            ...super.toDict(),
            start_location_uuid: this.startLocationUuid,
            end_location_uuid: this.endLocationUuid
        };

        if (this._contextPatch === null) {
            msg.trajectory_uuid = this.trajectoryUuid;
        } else {
            msg.trajectory = this.trajectory.toDict();
        }

        return msg;
    }

    static fromDict(dct) {
        return new MoveTrajectory(
            name= dct.name,
            type= dct.type,
            appendType= false,
            uuid= dct.uuid,
            startLocUuid= dct.start_location_uuid,
            endLocUuid = dct.end_location_uuid,
            trajectory= ('trajectory' in dct) ? Trajectory.fromDict(dct.trajectory) : null,
            trajectoryUuid= ('trajectory_uuid' in dct) ? dct.trajectory_uuid : null 
        );
    }

    /*
    * Accessor/modifier methods
    */

    context() {
        // Need to patch context when none exists
        const realContext = super.context;
        if (realContext === null) {
            if (this._contextPatch === null) {
                this._contextPatch = new ContextPatch();
            }
            return this._contextPatch;
        } else {
            if (this._contextPatch !== null) {
                this._contextPatch.updateContext(realContext);
                this._contextPatch = null;
            }
            return realContext;
        }
    }

    get startLocationUuid() {
        return this._startLocationUuid;
    }

    set startLocationUuid(value) {
        if (this._startLocationUuid !== value) {
            this._startLocationUuid = value;

            const t = this.context.getTrajectory(this.trajectoryUuid);
            if (t !== null) {
                t.startLocationUuid = this._startLocationUuid;
            }

            this.updatedAttribute('start_location_uuid','set');
        }
    }

    get endLocationUuid() {
        return this._endLocationUuid;
    }

    set endLocationUuid(value) {
        if (this._endLocationUuid !== value) {
            this._endLocationUuid = value;

            const t = this.context.getTrajectory(this.trajectoryUuid);
            if (t !== null) {
                t.endLocationUuid = this._endLocationUuid;
            }

            this.updatedAttribute('end_location_uuid','set');
        }
    }

    get trajectory() {
        return this.context.getTrajectory(this.trajectoryUuid);
    }

    set trajectory(value) {
        if (this._trajectoryUuid !== value.uuid) {
            this.context.deleteTrajectory(this.trajectoryUuid);
            this.context.addTrajectory(value);
            this.trajectoryUuid = value.uuid;
        }
    }

    get trajectoryUuid() {
        return this._trajectoryUuid;
    }

    set trajectoryUuid(value) {
        if (this.trajectoryUuid !== value) {
            if (value === null) {
                throw new Error('Id must be defined');
            }

            const traj = this.context.getTrajectory(value);
            if (traj === null && this._contextPatch === null) {
                throw new Error('Id must have a trajectory already in context');
            } else if (traj === null && this._contextPatch !== null) {
                this._contextPatch.addPendingTrajectory(
                    uuid=value,
                    startLoc=this.startLocationUuid,
                    endLoc=this.endLocationUuid
                );
            } else if (traj !== null) {
                if (traj.startLocationUuid !== this.startLocationUuid) {
                    throw new Error('Trajectory must have matching start location to primtive if set by id');
                }

                if (traj.endLocationUuid !== this.endLocationUuid) {
                    throw new Error('Trajectory must have matching end location to primitive if set by id');
                }
            }

            this._trajectoryUuid = value;
            this.updatedAttribute('trajectory_uuid','set');
        }
    }

    set(dct) {

        if ('start_location_uuid' in dct) {
            this.startLocationUuid = dct.start_location_uuid;
        }

        if ('end_location_uuid' in dct) {
            this.endLocationUuid = dct.end_location_uuid;
        }

        if ('trajectory' in dct) {
            this.trajectory = Trajectory.fromDict(dct.trajectory);
        }

        if ('trajectory_uuid' in dct) {
            this.trajectoryUuid = dct.trajectory_uuid;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    lateConstructUpdate() {
        const tmp = this.context; // force update context;

        super.lateConstructUpdate();
    }

    deepUpdate() {
        const tmp = this.context; // force update context

        super.deepUpdate();

        this.updatedAttribute('start_location_uuid','update');
        this.updatedAttribute('end_location_uuid','update');
        this.updatedAttribute('trajectory_uuid','update');
        this.updatedAttribute('trajectory','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('start_location_uuid','update');
        this.updatedAttribute('end_location_uuid','update');
        this.updatedAttribute('trajectory_uuid','update');
        this.updatedAttribute('trajectory','update');
    }
}


class ContextPatch {

    constructor(parent) {
        this._trajctories = {};
        this._parent = parent;
        this._pending = {};
    }

    get parent() {
        return this._parent;
    }

    get trajectories() {
        const traj = {
            ...this._trajctories,
            ...this._pending
        };
        return traj;
    }

    set trajectories(value) {
        for (const [uuid, t] of Object.entries(this._trajctories)) {
            t.removeFromCache();
        }
        this._trajctories = {};

        for (const [uuid, t] of Object.entries(value)) {
            this._trajctories[uuid] = t;
            t.parent = this;
        }
    }

    getTrajectory(uuid) {
        if (uuid in this._trajctories) {
            return this._trajctories[uuid];
        } else if (uuid in this._pending) {
            return 'pending';
        } else {
            return null;
        }
    }

    addTrajectory(trajectory) {
        trajectory.parent = this;
        this._trajctories[trajectory.uuid] = trajectory;
    }

    deleteTrajectory(uuid) {
        if (uuid in this._trajctories) {
            this._trajctories[uuid].removeFromCache();
            delete this._trajctories[uuid];
        } else if (uuid in this._pending) {
            delete this._pending[uuid];
        }
    }

    updateContext(context) {

        for (const [uuid, t] of Object.entries(this._trajctories)) {
            context.addTrajectory(t);
        }

        for (const [uuid, obj] of Object.entries(this._pending)) {
            const traj = context.getTrajectory(uuid);
            if (traj === null) {
                throw new Error('Pending trajectory was not found when context was assigned');
            }

            if (traj.startLocationUuid !== obj.startLoc) {
                throw new Error('Pending trajectory start location does not match one in context');
            }

            if (traj.endLocationUuid !== obj.endLoc) {
                throw new Error('Pending trajectory end location does not match one in context');
            }

            this._trajctories = {};
            this._pending = {};
        }
    }

    childChangedEvent(attributeTrace) {
        this.parent.childChangedEvent(attributeTrace);
    }

    addPendingTrajectory(uuid, startLoc, endLoc) {
        this._pending[uuid] = {startLoc, endLoc};
    }

}