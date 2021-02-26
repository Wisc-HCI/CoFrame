import { v4 as uuidv4 } from 'uuid';

import { Version, VersionUnwrapped } from '../ros/msg/version';


export class VersionTag {

    constructor(source, timestamp=null, uuid=null) {
        if (uuid !== null) {
            this.uuid = uuid;
        } else {
            this.uuid = this._generateUuid('version-tag');
        }
        this.timestamp = (timestamp !== null) ? timestamp : this._generateTimestamp();
        this.source = source
    }

    toDict() {
        return {
            type: 'version_tag',
            uuid: this.uuid,
            timestamp: this.timestamp,
            source: this.source
        };
    }

    toRos(wrap=true) {
        if (wrap) {
            return Version(
                this.timestamp, 
                this.uuid, 
                this.source
            );
        } else {
            return VersionUnwrapped(
                this.timestamp, 
                this.uuid, 
                this.source
            );
        }
    }

    static fromDict(dct) {
        return new VersionTag(
            source= dct.source,
            timestamp= dct.timestamp,
            uuid= dct.uuid
        );
    } 

    fromRos(msg) {
        return new VersionTag(
            source= msg.source,
            timestamp= msg.timestamp,
            uuid= msg.uuid
        );
    }

    static _generateUuid(type) {
        return `${type}-js-${uuidv4()}`;
    }

    static _generateTimestamp() {
        const currentTime = new Date();
        const secs = Math.floor(currentTime.getTime() / 1000);
        const nsecs = Math.round(1000000000*(currentTime.getTime()/1000-secs));
        return {secs, nsecs};
    }

}