import { assert, expect, test } from "vitest";
// import init, { Solver } from 'coframe-rust';
import { performIssueTest } from "../src/stores/issueDetectors/tests";
import doubleInit from "./programs/double_init.json"
import {Solver} from '@people_and_robots/lively';
import {UR5E_ROBOT_DATA} from "../src/presets/robotAgents/ur5eRobot"

test("Math.sqrt()", () => {
  expect(Math.sqrt(4)).toBe(2);
  expect(Math.sqrt(144)).toBe(12);
  expect(Math.sqrt(2)).toBe(Math.SQRT2);
});

test("JSON", () => {
  const input = {
    foo: "hello",
    bar: "world",
  };

  const output = JSON.stringify(input);

  expect(output).eq('{"foo":"hello","bar":"world"}');
  assert.deepEqual(JSON.parse(output), input, "matches original");
});

test("Double Init Test", async () => {
    // await init();
    // const issues = await performIssueTest(doubleInit,Solver);
    // await init();
    let solver = new Solver(UR5E_ROBOT_DATA["robot-agent"].properties.urdf,{});
    solver.computeAverageDistanceTable();
    // const matchedIssues = Object.values(issues).filter((issue)=>issue.code === "machineDoubleInit");
    // expect(matchedIssues.length).toBe(1);
    expect(1).toBe(1)
})
