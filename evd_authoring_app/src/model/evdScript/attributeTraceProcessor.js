export class AttributeTraceProcessor {

    constructor() {
        this._subscribers = {};
    }

    subscribeToType(type, callback) {
        if (! (type in this._subscribers)) {
            this._subscribers[type] = [];
        }

        this._subscribers[type].push(callback);
    }

    unsubscribeToType(type, callback) {
        if (! (type in this._subscribers)) {
            return false;
        }

        const idx = this._subscribers[type].indexOf(callback);
        if (idx >= 0) {
            this._subscribers[type].splice(idx,1);
            return true;
        } else {
            return false;
        }
    }

    processTrace(attributeTrace) {
        attributeTrace.forEach(element => {
            if (element.type in this._subscribers) {
                this.subscribers[element.type].forEach(cb => {
                    cb(element);
                });
            }
        });
    }
}