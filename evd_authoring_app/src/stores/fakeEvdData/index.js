/*
 * Fake data attempts to expess EvDscript's datastructure without
 * need for ROS connection. Useful for development and expectation
 * documentation.
 */


import arbitrary from './arbitrary';
import logical from './logical';
import template from './template';
import comm from './comm';

const fields = {
    arbitrary,
    logical,
    template,
    comm
};

export default fields