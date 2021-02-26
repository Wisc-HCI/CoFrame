
export class History {

    constructor(historyDepth=10) {
        this._history = [];
        this._historyDepth = historyDepth;
    }

    append(entry) {
        this._history.push(entry);
        if (this._history.length > this._historyDepth) {
            this._history.splice(0, 1);
        }
    }

    getCurrentEntry() {
        if (this._history.length > 0) {
            const idx = this._history.length - 1;
            return this._history[idx];
        } else {
            return null;
        }
    }

    getCurrentVersion() {
        if (this._history.length > 0) {
            const idx = this._history.length - 1;
            return this._history[idx].version;
        } else {
            return null;
        }
    }

    getPreviousVersion() {
        if (this._history.length > 0) {
            const idx = this._history.length - 2;
            return this._history[idx].version;
        } else {
            return null;
        }
    }

}