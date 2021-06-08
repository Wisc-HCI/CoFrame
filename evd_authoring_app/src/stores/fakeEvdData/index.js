/*
 * Fake data attempts to expess EvDscript's datastructure without
 * need for ROS connection. Useful for development and expectation
 * documentation.
 */



import arbitrary from './arbitrary';
import debugApp from './debugApp';
import template from './template';
import comm from './comm';

const fields = {
    arbitrary,
    debugApp,
    template,
    comm
};

export default fields