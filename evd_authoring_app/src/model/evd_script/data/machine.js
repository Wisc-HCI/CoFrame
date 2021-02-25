import { Node } from '../node';
import { NodeParser } from '../utility_functions';


export class MachineRecipe extends Node {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'machine-recipe.';
    }

    static fullTypeString() {
        return Node.fullTypeString() + MachineRecipe.typeString();
    }

    constructor(processTime=0, inputThingQuantities={}, outputThingQuantities={}, type='', name='', uuid=null, parent=null, appendType=true) {
        this._processTime = null,
        this._inputQuantities = null,
        this._outputQuantities = null

        super(
            type= (appendType) ? 'machine-recipe'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.processTime = processTime;
        this.inputThingQuantities = inputThingQuantities;
        this.outputThingQuantities = outputThingQuantities;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            process_time: this.processTime,
            input_thing_quantities: this.inputThingQuantities,
            output_thing_quantities: this.outputThingQuantities
        };
    }

    static fromDict(dct) {
        return new MachineRecipe(
            processTime= dct.process_time,
            inputThingQuantities= dct.input_thing_quantities,
            outputThingQuantities= dct.output_thing_quantities,
            type= dct.type,
            name= dct.name,
            uuid= dct.uuid,
            appendType= false
        );
    }

    /*
    * Data accessor/modifier methods
    */

    get processTime() {
        return this._processTime
    }

    set processTime(value) {
        if (this._processTime !== value) {
            this._processTime = value;
            this.updatedAttribute('process_time','set');
        }
    }

    get inputThingQuantities() {
        return this._inputQuantities;
    }

    set inputThingQuantities(value) {
        if (this._inputQuantities !== value) {
            this._inputQuantities = value;
            this.updatedAttribute('input_thing_quantities','set');
        }
    }

    addInputThingQuantitiy(thingType, quantity, override=false) {
        let verb = 'add';

        if (thingType in this._inputQuantities) {
            if (!override) {
                throw new Error('Thing quantity is about to be overrided, override is not allowed');
            } else {
                verb = 'update';
            }
        }

        this._inputQuantities[thingType] = quantity;
        this.updatedAttribute('input_thing_quantities',verb);
    }

    deleteInputThingQuantity(thingType) {
        if (! thingType in this._inputQuantities) {
            throw new Error(`No such ${thingType} in input quantities`);
        }

        delete this._inputQuantities[thingType];
        this.updatedAttribute('input_thing_quantities','delete');
    }

    getInputThingQuantity(thingType) {
        return this._inputQuantities[thingType];
    }

    get outputThingQuantities() {
        return this._outputQuantities;
    }

    set outputThingQuantities(value) {
        if (this._outputQuantities !== value) {
            this._outputQuantities = value;
            this.updatedAttribute('output_thing_quantities','set');
        }
    }

    addOutputThingQuantitiy(thingType, quantity, override=false) {
        let verb = 'add';

        if (thingType in this._outputQuantities) {
            if (!override) {
                throw new Error('Thing quantity is about to be overrided, override is not allowed');
            } else {
                verb = 'update';
            }
        }

        this._outputQuantities[thingType] = quantity;
        this.updatedAttribute('output_thing_quantities',verb);
    }

    deleteOutputThingQuantity(thingType) {
        if (! thingType in this._outputQuantities) {
            throw new Error(`No such ${thingType} in output quantities`);
        }

        delete this._outputQuantities[thingType];
        this.updatedAttribute('output_thing_quantities','delete');
    }

    getOutputThingQuantity(thingType) {
        return this._outputQuantities[thingType];
    }

    set(dct) {

        if ('process_time' in dct) {
            this.processTime = dct.process_time;
        }

        if ('input_thing_quantities' in dct) {
            this.inputThingQuantities = dct.input_thing_quantities;
        }

        if ('output_thing_quantities' in dct) {
            this.outputThingQuantities = dct.output_thing_quantities;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('process_time','update');
        this.updatedAttribute('input_thing_quantities','update');
        this.updatedAttribute('output_thing_quantities','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('process_time','update');
        this.updatedAttribute('input_thing_quantities','update');
        this.updatedAttribute('output_thing_quantities','update');
    }

}

export class Machine extends Node {

    /*
    * Data structure methods
    */ 

    static typeString() {
        return 'machine.';
    }

    static fullTypeString() {
        return Node.fullTypeString() + Machine.type();
    }

    constructor(inputRegions=null, outputRegions=null, recipe=null, type='', name='', uuid=null, parent=null, appendType=true) {

        this._inputRegions = null;
        this._outputRegions = null;
        this._machineType = null;
        this._recipe = null;

        super(
            type= (appendType) ? 'machine'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.inputRegions = inputRegions;
        this.outputRegions = outputRegions;
        this.recipe = (recipe !== null) ? recipe : new MachineRecipe();
    }

    toDict() {

        dctInputRegions = {};
        for (const [key, value] of Object.entries(this.inputRegions)) {
            dctInputRegions[key] = value.toDict();
        }

        dctOutputRegions = {};
        for (const [key, value] of Object.entries(this.outputRegions)) {
            dctOutputRegions[key] = value.toDict();
        }

        const msg = {
            ...super.toDict(),
            input_regions: dctInputRegions,
            output_regions: dctOutputRegions,
            recipe: this.recipe.toDict()
        };
        return msg;
    }

    static fromDict(dct) {

        let inputRegions = {};
        for (const [key, value] of Object.entries(dct.input_regions)) {
            inputRegions[key] = NodeParser(value);
        }

        let outputRegions = {};
        for (const [key, value] of Object.entries(dct.output_regions)) {
            outputRegions[key] = NodeParser(value);
        }

        return new Machine(
            inputRegions= inputRegions, 
            outputRegions= outputRegions, 
            recipe= MachineRecipe.fromDict(dct.recipe), 
            type= dct.type, 
            name= dct.name, 
            uuid= dct.uuid, 
            appendType=false
        );
    }

    toBlockly() {
        // TODO implement
        return {};
    }

    /*
    * Data accessor/modifier methods
    */

    get machineType() {
        return this._machineType;
    }

    get recipe() {
        return this._recipe;
    }

    set recipe(value) {
        if (this._recipe !== value) {
            if (value === null || value === undefined) {
                throw new Error('A recipe can be empty but it must exist!');
            }

            if (this._recipe != null) {
                this._recipe.removeFromCache();
            }

            this._recipe = value;
            this._recipe.parent = this;
            this.updatedAttribute('recipe','set');
        }
    }

    get inputRegions() {
        return this._inputRegions;
    }

    set inputRegions(value) {
        if (this._inputRegions !== value) {

            for ( const [type, region] of Object.entries(this._inputRegions)) {
                region.removeFromCache();
            }

            this._inputRegions = value;
            for (const [type, region] of Object.entries(this._inputRegions)) {
                region.parent = this;
            }

            this._computeType();
            this.updatedAttribute('input_regions','set');
        }
    }

    addInputRegion(thingType, region, override=false) {
        let verb = 'add';

        if (thingType in this._inputRegions) {
            if (!override) {
                throw new Error('Thing region is about to be overrrided, override is not allowed');
            } else {
                this._inputRegions[thingType].removeFromCache();
                verb = 'update';
            }
        }

        region.parent = this;
        this._inputRegions[thingType] = region;
        this._computeType();
        this.updatedAttribute('input_regions',verb,region.uuid);
    }

    deleteInputRegion(thingType) {
        if (! thingType in this._inputRegions) {
            throw new Error(`No such thing ${thingType} in input regions`);
        }

        const uuid = this._inputRegions[thingType].uuid;
        this._inputRegions[thingType].removeFromCache();
        delete this._inputRegions[thingType];
        this._computeType();
        this.updatedAttribute('input_regions','delete',uuid);
    }

    getInputRegion(thingType) {
        return this._inputRegions[thingType];
    }

    findInputRegionThingType(regionId) {
        let type = null;

        for (const [t, region] of Object.entries(this._inputRegions)) {
            if (region.uuid === regionId) {
                type = t;
                break;
            }
        }

        return type;
    }

    get outputRegions() {
        return this._outputRegions;
    }

    set outputRegions(value) {
        if (this._outputRegions !== value) {

            for ( const [type, region] of Object.entries(this._outputRegions)) {
                region.removeFromCache();
            }

            this._outputRegions = value;
            for (const [type, region] of Object.entries(this._outputRegions)) {
                region.parent = this;
            }

            this._computeType();
            this.updatedAttribute('output_regions','set');
        }
    }

    addOutputRegion(thingType, region, override=false) {
        let verb = 'add';

        if (thingType in this._outputRegions) {
            if (!override) {
                throw new Error('Thing region is about to be overrrided, override is not allowed');
            } else {
                this._outputRegions[thingType].removeFromCache();
                verb = 'update';
            }
        }

        region.parent = this;
        this._outputRegions[thingType] = region;
        this._computeType();
        this.updatedAttribute('output_regions',verb,region.uuid);
    }

    deleteOutputRegion(thingType) {
        if (! thingType in this._outputRegions) {
            throw new Error(`No such thing ${thingType} in output regions`);
        }

        const uuid = this._outputRegions[thingType].uuid;
        this._outputRegions[thingType].removeFromCache();
        delete this._outputRegions[thingType];
        this._computeType();
        this.updatedAttribute('output_regions','delete',uuid);
    }

    getOutputRegion(thingType) {
        return this._outputRegions[thingType];
    }

    findOutputRegionThingType(regionId) {
        let type = null;

        for (const [t, region] of Object.entries(this._outputRegions)) {
            if (region.uuid === regionId) {
                type = t;
                break;
            }
        }

        return type;
    }

    set(dct) {

        if ('input_regions' in dct) {
            let inputRegions = {};

            for (const [type, region] of Object.entries(dct.input_regions)) {
                inputRegions[type] = NodeParser(region);
            }

            this.inputRegions = inputRegions;
        }

        if ('output_regions' in dct) {
            let outputRegions = {};

            for (const [type, region] of Object.entries(dct.output_regions)) {
                outputRegions[type] = NodeParser(region);
            }

            this.oututRegions = outputRegions;
        }

        if ('recipe' in dct) {
            this.recipe = MachineRecipe.fromDict(dct.recipe);
        }

        super.set(dct);
    }

    /*
    * Cache methods
    */

    removeFromCache() {

        if (this.inputRegions !== null) {
            for ( const [type, region] of Object.entries(this.inputRegions)) {
                region.removeFromCache();
            }
        }

        if (this.outputRegions !== null) {
            for ( const [type, region] of Object.entries(this.outputRegions)) {
                region.removeFromCache();
            }
        }

        this.recipe.removeFromCache();
        super.removeFromCache();
    }

    addToCache() {

        if (this.inputRegions !== null) {
            for ( const [type, region] of Object.entries(this.inputRegions)) {
                region.addToCache();
            }
        }

        if (this.outputRegions !== null) {
            for ( const [type, region] of Object.entries(this.outputRegions)) {
                region.addToCache();
            }
        }
        
        this.recipe.addToCache();
        super.addToCache();
    }

    /*
    * Update methods
    */

    lateConstructUpdate() {

        if (this.inputRegions !== null) {
            for ( const [type, region] of Object.entries(this.inputRegions)) {
                region.lateConstructUpdate();
            }
        }

        if (this.outputRegions !== null) {
            for ( const [type, region] of Object.entries(this.outputRegions)) {
                region.lateConstructUpdate();
            }
        }

        this.recipe.lateConstructUpdate();

        super.lateConstructUpdate();
    }

    deepUpdate() {

        if (this.inputRegions !== null) {
            for ( const [type, region] of Object.entries(this.inputRegions)) {
                region.deepUpdate();
            }
        }

        if (this.outputRegions !== null) {
            for ( const [type, region] of Object.entries(this.outputRegions)) {
                region.deepUpdate();
            }
        }

        this.recipe.deepUpdate();

        super.deepUpdate();

        this.updatedAttribute('machine_type','update');
        this.updatedAttribute('input_regions','update');
        this.updatedAttribute('output_regions','update');
        this.updatedAttribute('recipe','update');

    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('machine_type','update');
        this.updatedAttribute('input_regions','update');
        this.updatedAttribute('output_regions','update');
        this.updatedAttribute('recipe','update');
    }

    /* 
    * Utility Methods
    */

    _computeType() {
        let type = null;

        if (this.inputRegions === null && this.outputRegions === null) {
            type = 'useless';
        } else if (this.inputRegions === null && this.outputRegions !== null) {
            if (Object.values(this.outputRegions).length > 0) {
                type = 'generator';
            } else {
                type = 'useless';
            }
        } else if (this.inputRegions !== null && this.outputRegions === null) {
            if (Object.values(this.inputRegions).length > 0) {
                type = 'consumer';
            } else {
                type = 'useless';
            }
        } else {
            type = 'transformer';
        }

        this._machineType = type;
        this.updatedAttribute('machine_type','set');
    }
}
