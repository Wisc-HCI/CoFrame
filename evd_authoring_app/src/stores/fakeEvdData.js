let waypoints = [];
for (let i=0; i<20; i++) {
    waypoints.push({
        uuid: i,
        name: `Waypoint-${i}`
    });
}

let regions = [];
for (let i=0; i<30; i++) {
    regions.push({
        uuid: i,
        name: `Region-${i}`,
        canDelete: i % 3,
        canEdit: i % 3
    });
}



let locations = [];
for (let i=0; i<20; i++) {
    locations.push({
        uuid: i,
        name: `Location-${i}`,
        canDelete: ! (i === 3),
        canEdit: ! (i % 4 === 0)
    });
}

let machines = [];
for (let i=0; i<30; i++) {
    machines.push({
        uuid: i,
        name: `Machine-${i}`,
        canDelete: i % 3,
        canEdit: i % 3,
        mesh: 'default.stl',
        inputs: [],
        outputs: []
    });
}

let thingTypes = [];
for (let i=0; i<5; i++) {
    thingTypes.push({
        uuid: i,
        name: `Thing-Type-${i}`,
        canDelete: false,
        mesh: 'default.stl'
    });
}

let things = [];
for (let i=0; i<10; i++) {
    things.push({
        uuid: i,
        name: `Thing-${i}`,
        canDelete: false,
        canEdit: true
    });
}

const fields = {waypoints,regions,locations,machines,thingTypes,things}

export default fields