import { Node } from '../node';
import { Pose, Position, Orientation } from './geometry';


export class TraceDataPoint extends Pose {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'trace-data-point.';
    }

    static fullTypeString() {
        return Pose.fullTypeString() + TraceDataPoint.typeString();
    }

    constructor(position=null, orientation=null, grade=0, type='', name='', uuid=null, parent=null, appendType=true) {
        this._grade = null;

        super(
            position= position,
            orientation= orientation,
            type= (appendType) ? 'trace-data-point.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.grade = grade;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            grade: this.grade
        };
        return msg;
    }

    static fromDict(dct) {
        return new TraceDataPoint(
            position= Position.fromDict(dct.position),
            orientation= Orientation.fromDict(dct.orientation),
            grade= dct.grade,
            type= dct.type,
            name= dct.name,
            uuid= dct.uuid,
            appendType= false
        );
    }

    /*
    * Data accessor/modifier methods
    */

    get grade() {
        return this._grade;
    }

    set grade(value) {
        if (this._grade != value) {
            this._grade = grade;
            this.updatedAttribute('grade','set');
        }
    }

    set(dct) {

        if ('grade' in dct) {
            this.grade = dct.grade;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('grade','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('grade','update');
    }
}

export class Trace extends Node {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'trace.';
    }

    static fullTypeString() {
        return Node.fullTypeString() + Trace.typeString();
    }

    constructor(eePath=null, data={}, jPaths=[], tPaths=[], cPaths=[], time=0, type='', name='', uuid=null, parent=null, appendType=true) {

        this._data = {};
        this._time = null;
        this._endEffectorPath = null;
        this._jointPaths = null;
        this._toolPaths = null;
        this._componentPaths = null;

        super(
            type= (appendType) ? 'trace.'+type : type,
            name= name,
            uuuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.data = data;
        this.time = time;
        this.endEffectorPath = eePath;
        this.jointPaths = jPaths;
        this.toolPaths = tPaths;
        this.componentPaths = cPaths;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            data: this.data,
            time: this.time,
            end_effector_path: this.endEffectorPath,
            joint_paths: this.jointPaths,
            tool_paths: this.toolPaths,
            component_paths: this.componentPaths
        };
        return msg;
    }

    static fromDict(dct) {

        let data = {};
        for (const [key, value] of Object.entries(dct.data)) {
            data[key] = value.map(v => TraceDataPoint.fromDict(v));
        }

        return new Trace(
            uuid= dct.uuid,
            type= dct.type,
            name= dct.name,
            appendType= false,
            data= data,
            eePath= dct.end_effector_path,
            jPaths= dct.joint_paths,
            tPaths= dct.tool_paths,
            cPaths= dct.component_paths,
            time= dct.time
        );       
    }

    /*
    * Data accessor/modifier methods
    */

    get data() {
        return this._data;
    }

    set data(value) {
        if (this._data !== value) {

            for (const [key,l] of Object.entries(this._data)) {
                for (const d in l) {
                    d.removeFromCache();
                }
            }

            this._data = value;
            for (const [key,l] of Object.entries(this._data)) {
                for (const d in l) {
                    d.parent = this;
                }
            }
        }
    }

    get time() {
        return this._time;
    }

    set time(value) {
        if (this._time !== value) {
            this._time = value;
            this.updatedAttribute('time','set');
        }
    }

    get endEffectorPath() {
        return this._endEffectorPath;
    }

    set endEffectorPath(value) {
        if (this._endEffectorPath !== value) {
            this._endEffectorPath = value;
            this.updatedAttribute('end_effector_path','set');
        }
    }

    get jointPaths() {
        return this._jointPaths;
    }

    set jointPaths(value) {
        if (this._jointPaths !== value) {
            this._jointPaths = value;
            this.updatedAttribute('joint_paths','set');
        }
    }

    get toolPaths() {
        return this._toolPaths;
    }

    set toolPaths(value) {
        if (this._toolPaths !== value) {
            this._toolPaths = value;
            this.updatedAttribute('tool_paths','set');
        }
    }

    get componentPaths() {
        return this._componentPaths;
    }

    set componentPaths(value) {
        if (this._componentPaths !== value) {
            this._componentPaths = value;
            this.updatedAttribute('component_paths','set');
        }
    }

    addDataPoint(dp, group) {
        dp.parent = this;

        if (! group in this._data) {
            this._data[group] = [];
        }
        this._data[group].push(dp);

        this.updatedAttribute('data','add');
    }

    deleteDataPoint(uuid, group) {
        let idx = null;
        for (let i=0; i<this._data[group].length; i++) {
            if (this._data[group][i].uuid === uuid) {
                idx = i;
                break;
            }
        }

        if (idx !== null) {
            this._data[group][idx].removeFromCache();
            delete this._data[group][idx]
            this.updatedAttribute('data','delete');
        }

    }

    getDataPoint(uuid, group) {
        for (const d in this._data[group]) {
            if (d.uuid === uuid) {
                return d;
            }
        }
        return null;
    }

    set(dct) {

        if ('data' in dct) {
            this.data = dct.data;
        }

        if ('time' in dct) {
            this.time = dct.time;
        }

        if ('end_effector_path' in dct) {
            this.endEffectorPath = dct.end_effector_path;
        }

        if ('joint_paths' in dct) {
            this.jointPaths = dct.joint_paths;
        }

        if ('tool_paths' in dct) {
            this.toolPaths = dct.tool_paths;
        }

        if ('component_paths' in dct) {
            this.componentPaths = dct.component_paths;
        }

        super.set(dct);
    }

    /*
    * Cache Methods
    */

    removeFromCache() {

        for (const [key, l] of Object.entries(this._data)) {
            for (const d in l) {
                l.removeFromCache();
            }
        }

        super.removeFromCache();
    }

    addToCache() {

        for (const [key, l] of Object.entries(this._data)) {
            for (const d in l) {
                l.addToCache();
            }
        }

        super.addToCache();
    }

    /*
    * Children methods
    */

    deleteChild(uuid) {
        let group = null;
        for (const [key, l] of Object.entries(this._data)) {
            for (const d in l) {
                if (d.uuid === uuid) {
                    group = key;
                    break;
                }
            }

            if (group !== null) {
                break;
            }
        }

        if (group !== null) {
            this.deleteDataPoint(uuid, group);
            return true;
        } else {
            return false;
        }
    }

    /*
    * Update Methods
    */

    lateConstructUpdate() {

        for (const [key, l] of Object.entries(this._data)) {
            for (const d in l) {
                d.lateConstructUpdate();
            }
        }

        super.lateConstructUpdate();
    }

    deepUpdate() {

        for (const [key, l] of Object.entries(this._data)) {
            for (const d in l) {
                d.deepUpdate();
            }
        }

        super.deepUpdate();

        this.updatedAttribute('data','update');
        this.updatedAttribute('time','update');
        this.updatedAttribute('end_effector_path','update');
        this.updatedAttribute('joint_paths','update');
        this.updatedAttribute('tool_paths','update');
        this.updatedAttribute('component_paths','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('data','update');
        this.updatedAttribute('time','update');
        this.updatedAttribute('end_effector_path','update');
        this.updatedAttribute('joint_paths','update');
        this.updatedAttribute('tool_paths','update');
        this.updatedAttribute('component_paths','update');
    }
}