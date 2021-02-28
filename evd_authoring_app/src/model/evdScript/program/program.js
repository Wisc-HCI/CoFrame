import { Task } from './task';
import { Environment } from '../environment/environment';
import { NodeParser } from '../utilityFunctions';

export class Program extends Task {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'program.';
    }

    static fullTypeString() {
        return Task.fullTypeString() + Program.typeString();
    }

    constructor(primitives=[], environment=null, changesCB=null, name='', type='', 
                uuid=null, appendType=true) 
    {
        if (environment === null) {
            environment = new Environment();
        }

        if (! environment instanceof Environment) {
            throw new Error('Program level context must be an environment');
        }

        super(
            primitives,
            (appendType) ? 'program.'+ type : type,
            name,
            uuid,
            null,
            appendType
        );

        this.changesCb = changesCB;
        this._environment = null;

        this.environment = environment;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            environment: this.environment.toDict()
        };
        return msg;
    }

    static fromDict(dct) {
        return new Program(
            dct.primitives.map(p => NodeParser(p)),
            NodeParser(dct.environment),
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false 
        );
    }

    /*
    * Access/modifier methods
    */

    get context() {
        // Alias
        return this.environment;
    }

    set context(value) {
        // Alias
        this.environment = value;
    }

    get environment() {
        return this._environment;
    }

    set environment(value) {
        if (this._environment !== value) {
            if (! value instanceof Environment) {
                throw new Error('Program level context must be an environemnt');
            }

            if (this._environment !== null) {
                this._environment.removeFromCache();
            }
            this._environment = value;
            this._environment.parent = this;

            this.updatedAttribute('context','set');
            this.updatedAttribute('environment','set');
        }
    }

    set(dct) {

        if ('environment' in dct) {
            this.environment = Environment.fromDict(dct.environment);
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    lateConstructUpdate() {
        this.environment.lateConstructUpdate();
        super.lateConstructUpdate();
    }

    deepUpdate() {
        this.environment.deepUpdate();
        super.deepUpdate();

        this.updatedAttribute('context','update');
        this.updatedAttribute('environment','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('context','update');
        this.updatedAttribute('environment','update');
    }

    /*
    * Utility methods
    */

    childChangedEvent(attributeTrace) {
        if (this.changesCb !== null) {
            attributeTrace.push(this._childChangedEventMsg(null,'callback'));
            this.changesCb(attributeTrace);
        }
    }

    updatedAttribute(attribute, verb, childUuid=null) {
        const event = [this._childChangedEventMsg(attribute, verb, childUuid)];

        if (this.changesCb !== null) {
            this.changesCb(event);
        }
    }
}