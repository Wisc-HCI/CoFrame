import { Container } from './container';

const TestNodeParser = (exactType, dct) => {

    let node = null;

    if (exactType.includes('container') && 'item_type' in dct) {
        if (exactType === `container<${dct.item_type}>`) {
            node = Container.fromDict(dct);
        } else {
            throw new Error('Illegal container format when attempting to run node parser');
        }
    }

    return node;
}

export {
    Container,
    TestNodeParser
};