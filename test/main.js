// import {Solver} from '@people_and_robots/lively';
import { performIssueTest } from "../src/stores/issueDetectors/tests";
import ur5 from "./programs/ur5.json"

import { quaternionFromEuler } from "../src/helpers/geometry";



console.log('In Main',quaternionFromEuler([0,0,Math.PI/2],"sxyz"))

await performIssueTest(ur5)
