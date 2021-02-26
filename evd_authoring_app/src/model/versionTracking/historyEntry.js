import { VersionTag } from './versionTag';

export class HistoryEntry {

    constructor(action, changeDct, versionTag=null, snapshotDct=null, source=null) {
        this.action = action;
        this.changes = changeDct;
        this.snapshot = snapshotDct;

        if (versionTag !== null) {
            this.version = versionTag;
        } else {
            if (source !== null) {
                this.version = new VersionTag(source);
            } else {
                throw new Error('Must either provide version tag or source for new tag');
            }
        }
    }

    toDict() {
        return {
            type: 'history-entry',
            action: this.action,
            change: this.changes,
            version_tag: this.version.toDict(),
            snapshot: this.snapshot
        };
    }

    static fromDict(dct) {
        return new HistoryEntry(
            action= dct.action,
            changeDct= dct.change,
            versionTag= dct.version_tag,
            snapshotDct= dct.snapshot
        );
    }

}