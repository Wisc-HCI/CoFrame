// import {Solver} from '@people_and_robots/lively';
import { performIssueTest } from "../src/stores/issueDetectors/tests";
import ur5 from "./programs/ur5.json"

console.log('In Main')

await performIssueTest(ur5)
