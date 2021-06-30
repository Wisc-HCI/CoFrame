/*
 * Fake data attempts to expess EvDscript's datastructure without
 * need for ROS connection. Useful for development and expectation
 * documentation.
 */



import arbitrary from './arbitrary';
import template from './template';
import comm from './comm';

const fields = {
    arbitrary,
    template,
    comm
};

export default fields