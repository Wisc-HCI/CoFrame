import { getEvdCacheObject } from './cache';

import { DataNodeParser } from './data';
import { EnvironmentNodeParser } from './environment';
import { ProgramNodeParser } from './program';
import { TestNodeParser } from './test';

import { Node } from './node';
import { Context } from './context';


export const NodeParser = (dct) => {

    let node = undefined;

    // Check if object already is in the cache
    try {
        node = getEvdCacheObject().get(dct.uuid);
    } catch(ex) {
        node = null;
    }

    // must create new object
    const type = dct.type.split('.');
    const exactType = type[type.length - 2];

    node = DataNodeParser(exactType, dct);
    if (node !== null) {
        return node;
    }

    node = EnvironmentNodeParser(exactType, dct);
    if (node !== null) {
        return node;
    }

    node = ProgramNodeParser(exactType, dct);
    if (node !== null) {
        return node;
    }

    node = TestNodeParser(exactType, dct);
    if (node !== null) {
        return node;
    }

    if (exactType === 'node') {
        node = Node.fromDict(dct);
    } else if (exactType === 'context') {
        node = Context.fromDict(dct);
    } else {
        throw new Error(`Could not parse object supplied with type: ${exactType}`);
    }

    return node;
};

export const getExactType = (node) => {
    const type = node.type.split('.');
    const exactType = type[type.length - 2];
    return exactType;
}