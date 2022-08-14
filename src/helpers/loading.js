import { XMLParser } from "fast-xml-parser";
import { generateUuid } from "../stores/generateUuid";
import { instanceTemplateFromSpec } from "simple-vp";
import agentTypes from "../stores/typeInfo/agents";
import collisionTypes from "../stores/typeInfo/collision";
import sceneObjects from "../stores/typeInfo/sceneObjects";
import { quaternionFromEuler, quaternionVecToObject } from "./geometry";
import { meshType } from "../stores/typeInfo/mesh";

const capitalize = (value) => value[0].toUpperCase() + value.substring(1);

const originToPose = (origin) => {
  const posVec = origin.xyz.split(" ").map(Number);
  const rpyVec = origin.rpy.split(" ").map(Number);

  return {
    position: { x: posVec[0], y: posVec[1], z: posVec[2] },
    rotation: quaternionVecToObject(quaternionFromEuler(rpyVec, "sxyz")),
  };
};

const childrenPaths = [
  "robot.link",
  "robot.joint",
  "robot.link.collision",
  "robot.link.visual",
];

export const robotDataFromUrdf = (urdf, relativeTo) => {
  const parser = new XMLParser({
    ignoreNameSpace: true,
    ignoreAttributes: false,
    allowBooleanAttributes: true,
    attributeNamePrefix: "",
    isArray: (name, jpath, isLeafNode, isAttribute) =>
      childrenPaths.includes(jpath),
  });
  const data = parser.parse(urdf);
  console.log(data.robot);

  const robotId = generateUuid("robotAgentType");
  const robotName = capitalize(data.robot.name ? data.robot.name : "robot");

  let robot = {
    ...instanceTemplateFromSpec(
      "robotAgentType",
      agentTypes.robotAgentType,
      false
    ),
    id: robotId,
    name: robotName,
  };
  robot.properties.relativeTo = relativeTo;

  let allData = {
    [robotId]: robot,
  };

  // Find the root node. This is the link that is not referenced as a child of any joint.
  let root = null;
  data.robot.link.some((linkData) => {
    const hasParent = data.robot.joint.some(
      (jointData) => jointData.child.link === linkData.name
    );
    if (hasParent) {
      return false;
    } else {
      root = linkData;
      return true;
    }
  });
  console.log("root", root);

  // Al
  let jointLimit = {};
  let initialJointState = {};
  let linkNames = [];

  // Define a stack to iterate through
  let processStack = [
    { type: "link", data: root, parent: robot, relativeTo: robotId },
  ];

//   let jointCount = 0

  while (processStack.length > 0) {
    let toProcess = processStack.pop();
    if (toProcess.type === "link") {
      const linkId = generateUuid("linkType");
      let link = {
        ...instanceTemplateFromSpec("linkType", sceneObjects.linkType, false),
        id: linkId,
        name: toProcess.data.name,
        raw: toProcess.data,
      };
      link.properties.relativeTo = toProcess.relativeTo;
      link.properties.agent = robotId;
      if (toProcess.parent.id !== robotId) {
        console.log('non-root link',toProcess)
        const origin = originToPose(toProcess.parent.origin);
        link.properties.position = origin.position;
        link.properties.rotation = origin.rotation;
      }
      

      // Create a new collision body entry and assign it to the link's collision property.
      const collisionBodyId = generateUuid("collisionBodyType");
      let collisionBody = {
        ...instanceTemplateFromSpec(
          "collisionBodyType",
          collisionTypes.collisionBodyType,
          false
        ),
        id: collisionBodyId,
        name: `${link.name}-collisionbody`,
      };

      linkNames.push(link.name);

      link.properties.collision = collisionBodyId;

      // For each of the shapes in the collision, add a collisionShape and link it to the body
      let components = []
      toProcess.data.collision?.forEach((rawCollisionShape,idx) => {
        const collisionType = rawCollisionShape.geometry.cylinder ? 'cylinder'
            : rawCollisionShape.geometry.capsule ? 'capsule'
            : rawCollisionShape.geometry.box ? 'cube'
            : rawCollisionShape.geometry.sphere ? 'sphere'
            : 'mesh'
        if (collisionType !== 'mesh') {
            const collisionShapeId = generateUuid('collisionShapeType');
            let collisionShape = {...instanceTemplateFromSpec('collisionShapeType',collisionTypes.collisionShapeType,false),id:collisionShapeId,name:`${link.name}-${collisionType}-${idx}`};
            collisionShape.properties.keyword = collisionType;
            const collisionShapeOrigin = originToPose(rawCollisionShape.origin)
            collisionShape.properties.position = collisionShapeOrigin.position;
            collisionShape.properties.rotation = collisionShapeOrigin.rotation;
            allData[collisionShapeId] = collisionShape;
            components.push(collisionShapeId);
        }
      });

      collisionBody.properties.componentShapes = components;

      // Create a new mesh entry and assign it to the mesh property of the link.
      if (toProcess.data.visual?.length > 0) {
        let meshId = generateUuid("meshType");
        let mesh = {
          ...instanceTemplateFromSpec("meshType", meshType, false),
          id: meshId,
          name: `${link.name}-mesh`,
        };
        mesh.properties.keyword =
          toProcess.data.visual[0]?.geometry?.mesh?.filename;
        const meshOrigin = originToPose(toProcess.data.visual[0]?.origin);
        mesh.properties.position = meshOrigin.position;
        mesh.properties.rotation = meshOrigin.rotation;
        link.properties.mesh = meshId;
        allData[meshId] = mesh;
      }

      // Add all the data to the set
      allData[linkId] = link;
      allData[collisionBodyId] = collisionBody;

      // Find all joints that have this link as a parent and add them to the process stack
      data.robot.joint.forEach((rawJoint) => {
        if (rawJoint.parent.link === link.name) {
          processStack.push({
            type: "joint",
            data: rawJoint,
            parent: link,
            relativeTo: link.id,
          });
        }
      });
    } else if (toProcess.type === "joint") {
        // Find the child link and add it to the stack
        data.robot.link.some(rawLink=>{
            if (rawLink.name === toProcess.data.child.link) {
                processStack.push({
                    type: 'link',
                    data: rawLink,
                    parent: toProcess.data,
                    relativeTo: toProcess.relativeTo
                })
                return true
            }
            return false
        })
        if (toProcess.data.type !== 'fixed') {
            const lower = Number(toProcess.data.limit.lower);
            const upper = Number(toProcess.data.limit.upper);
            const mid = (lower + upper) / 2;
            initialJointState[toProcess.data.name] = mid;
            jointLimit[toProcess.data.name] = {lower, upper};
        }
        // jointCount += 1
        
    }
  }

  let pinchPointPairLinks = [];
  linkNames.forEach(link1=>{
    linkNames.forEach(link2=>{
      if (link1 !== link2) {
        pinchPointPairLinks.push({link1,link2})
      }
    })
  })

  allData[robotId].properties.initialJointState = initialJointState;
  allData[robotId].properties.jointLimit = jointLimit;
  allData[robotId].properties.pinchPointPairLinks = pinchPointPairLinks;

  console.log(allData);

  return {};
};
