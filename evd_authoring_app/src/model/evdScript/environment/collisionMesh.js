import { Node } from '../node';
import { Pose } from '../data/geometry';


export class CollisionMesh extends Node {

    /*
    * Constants
    */

    static GOOD_STATE = "good";
    static WARN_STATE = "warn";
    static ERROR_STATE = "error";

    /*
    * Data structure methods
    */

    static typeString() {
        return 'collision-mesh.';
    }

    static fullTypeString() {
        return Node.fullTypeString() + CollisionMesh.typeString();
    }

    constructor(meshId=null, poseOffset=null, link='', type='', name='', parent=null,
                uuid=null, appendType=true) 
    {
        super(
            (appendType) ? 'collision-mesh.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );

        this._meshId = null;
        this._poseOffset = null;
        this._link = null;

        this.meshId = meshId;
        this.poseOffset = poseOffset;
        this.link = link;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            mesh_id: this.meshId,
            pose_offset: this.poseOffset.toDict(),
            link: this.link
        };
        return msg;
    }

    static fromDict(dct) {
        return new CollisionMesh(
            dct.mesh_id,
            Pose.fromDict(dct.pose_offset),
            dct.link,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }

    /*
    * Data accessor/modifier methods
    */

    get meshId() {
        return this._meshId;
    }

    set meshId(value) {
        if (this._meshId !== value) {
            this._meshId = value;
            this.updatedAttribute('mesh_id','set');
        }
    }

    get poseOffset() {
        return this._poseOffset;
    }

    set poseOffset(value) {
        if (this._poseOffset !== value) {
            if (value === null) {
                throw new Error('pose offset cannot be null');
            }

            if (this._poseOffset !== null) {
                this._poseOffset.removeFromCache();
            }

            this._poseOffset = value;
            if (this._poseOffset !== null) {
                this._poseOffset.parent = this;
            }

            this.updatedAttribute('pose_offset','set');
        }
    }

    get link() {
        return this._link;
    }

    set link(value) {
        if (this._link !== value) {
            this._link = value;
            this.updatedAttribute('link','set');
        }
    }

    set(dct) {

        if ('mesh_id' in dct) {
            this.meshId = dct.mesh_id;
        }

        if ('pose_offset' in dct) {
            this.poseOffset = Pose.fromDict(dct.pose_offset);
        }

        if ('link' in dct) {
            this.link = dct.link;
        }

        super.set(dct);
    }

    /*
    * Cache methods
    */

    removeFromCache() {
        this.poseOffset.removeFromCache();
        super.removeFromCache();
    }

    addToCache() {
        this.poseOffset.addToCache();
        super.addToCache();
    }

    /*
    * Update methods
    */

    lateConstructUpdate() {
        this.poseOffset.lateConstructUpdate();
        super.lateConstructUpdate();
    }

    deepUpdate() {
        this.poseOffset.deepUpdate();

        super.deepUpdate();

        this.updatedAttribute('mesh_id','update');
        this.updatedAttribute('pose_offset','update');
        this.updatedAttribute('link','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('mesh_id','update');
        this.updatedAttribute('pose_offset','update');
        this.updatedAttribute('link','update');
    }
}