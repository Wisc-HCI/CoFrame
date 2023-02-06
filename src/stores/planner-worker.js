import * as Comlink from 'comlink';
import { performCompileProcess } from './compiling/performCompileProcess';
// import { Solver } from '@people_and_robots/lively';
Comlink.expose({
    performCompileProcess,
    // test:()=>'Comlink Test Passed',
    // Solver
})

// onmessage = (e) => {
//     console.log('Message received from main script');
//     // const result = performCompileProcess(e.data);
//     console.log('Posting message back to main script');
//     // postMessage(result)
// }