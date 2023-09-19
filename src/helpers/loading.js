import { XMLParser } from "fast-xml-parser";
import { generateUuid } from "../stores/generateUuid";
import { instanceTemplateFromSpec } from "open-vp";
import agentTypes from "../stores/typeInfo/agents";
import collisionTypes from "../stores/typeInfo/collision";
import sceneObjects from "../stores/typeInfo/sceneObjects";
import { quaternionFromEuler, quaternionVecToObject } from "./geometry";
import { meshType } from "../stores/typeInfo/mesh";
import { Solver } from "@people_and_robots/lively";

const capitalize = (value) => value[0].toUpperCase() + value.substring(1);

const originToPose = (origin) => {
  const posVec = origin?.xyz ? origin.xyz.split(" ").map(Number) : [0, 0, 0];
  const rpyVec = origin?.rpy ? origin.rpy.split(" ").map(Number) : [0, 0, 0];

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

  const solver = new Solver(urdf, {});

  console.log({ links: solver.links, joints: solver.joints });

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
  robot.properties.urdf = urdf;
  let root = solver.links[0].name;
  robot.properties.root = root;
  console.log(robot.properties)

  let jointLinkMap = {};
  solver.joints.forEach(j=>{
    jointLinkMap[j.name] = j.childLink
  })
  robot.properties.jointLinkMap = jointLinkMap;

  let linkParentMap = {};
  solver.links.forEach((l,i)=>{
    linkParentMap[l.name] = getParentLink(l.parentJoint,solver.joints,robotId,i>0?solver.links[i-1].name:robotId)
  })
  robot.properties.linkParentMap = linkParentMap;

  let allData = {
    [robotId]: robot,
  };

  // Find the root node. This is the link that is not referenced as a child of any joint.
  

  const initialState = solver.currentState;
  console.log("initialState", initialState);

  let initialJointState = {};
  let linkNames = [];
  let initialJoints = {};
  solver.joints.forEach(j=>initialJoints[j.name]=initialState.joints[j.name])

  solver.links.forEach((link,linkIdx) => {
    console.log(link.name);
    linkNames.push(link.name);
    initialJointState = initialState.joints;
    let linkMeshId = null;
    // Create meshTypes for each visul mesh. This assumes only one visual mesh per link
    link.visuals.forEach((linkVisual) => {
      if (linkVisual.type === "Mesh") {
        let meshId = generateUuid("meshType");
        let mesh = {
          ...instanceTemplateFromSpec("meshType", meshType, false),
          id: meshId,
          name: `${link.name}-mesh`,
        };
        mesh.properties.keyword = linkVisual.filename;
        mesh.properties.position = {
          x: linkVisual.localTransform.translation[0],
          y: linkVisual.localTransform.translation[1],
          z: linkVisual.localTransform.translation[2],
        };
        mesh.properties.rotation = {
          x: linkVisual.localTransform.rotation[0],
          y: linkVisual.localTransform.rotation[1],
          z: linkVisual.localTransform.rotation[2],
          w: linkVisual.localTransform.rotation[3],
        };
        mesh.properties.scale = {
          x: linkVisual.x,
          y: linkVisual.y,
          z: linkVisual.z,
        };
        linkMeshId = meshId;
        allData[meshId] = mesh;
      }
    });
    // Create collisionShapes for each collision entry.
    let collisions = [];
    link.collisions.forEach((linkCollision,idx) => {
      const collision = linkCollisionToCollisionShape(linkCollision,idx,robotId);
      allData[collision.id] = collision;
      collisions.push(collision.id)
    });

    // Create collisionBody for the link.
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
    collisionBody.properties.componentShapes = collisions;
    allData[collisionBodyId] = collisionBody;
    
    const linkId = link.name;
    let linkData = {
      ...instanceTemplateFromSpec("linkType", sceneObjects.linkType, false),
      id: linkId,
      name: link.name,
      raw: link,
    };

    linkData.properties.collision = collisionBodyId;
    linkData.properties.mesh = linkMeshId;

    linkData.properties.relativeTo = getParentLink(link.parentJoint,solver.joints,robotId,linkIdx>0?solver.links[linkIdx-1].name:robotId);
    linkData.properties.frameKey = link.name;
    linkData.properties.agent = robotId;
    linkData.properties.position = {
      x: initialState.frames[link.name].local.translation[0],
      y: initialState.frames[link.name].local.translation[1],
      z: initialState.frames[link.name].local.translation[2],
    };
    linkData.properties.rotation = {
      x: initialState.frames[link.name].local.rotation[0],
      y: initialState.frames[link.name].local.rotation[1],
      z: initialState.frames[link.name].local.rotation[2],
      w: initialState.frames[link.name].local.rotation[3],
    };
    allData[linkId] = linkData
  });

  let pinchPointPairLinks = [];
  linkNames.forEach((link1) => {
    linkNames.forEach((link2) => {
      if (link1 !== link2) {
        pinchPointPairLinks.push({ link1, link2 });
      }
    });
  });

  let jointLimit = {};
  solver.joints.forEach(joint=>{
    if (joint.jointType === 'rotational') {
      jointLimit[joint.name] = {upper:joint.upperBound,lower:joint.lowerBound}
    }
  })

  allData[robotId].properties.initialJointState = initialJoints;
  allData[robotId].properties.jointLimit = jointLimit;
  allData[robotId].properties.pinchPointPairLinks = pinchPointPairLinks;
  allData[robotId].properties.linkParentMap = linkParentMap;

  console.log(allData);

  return allData;
};

const linkCollisionToCollisionShape = (linkCollision,idx,linkName) => {
  const collisionType =
    linkCollision.type === "Box" ? "cube" : linkCollision.type.toLowerCase();
  if (collisionType !== "mesh") {
    const collisionShapeId = generateUuid("collisionShapeType");
    let collisionShape = {
      ...instanceTemplateFromSpec(
        "collisionShapeType",
        collisionTypes.collisionShapeType,
        false
      ),
      id: collisionShapeId,
      name: `${linkName}-${collisionType}-${idx}`,
    };
    collisionShape.properties.keyword = collisionType;
    collisionShape.properties.position = {
      x: linkCollision.localTransform.translation[0],
      y: linkCollision.localTransform.translation[1],
      z: linkCollision.localTransform.translation[2],
    };
    collisionShape.properties.rotation = {
      x: linkCollision.localTransform.rotation[0],
      y: linkCollision.localTransform.rotation[1],
      z: linkCollision.localTransform.rotation[2],
      w: linkCollision.localTransform.rotation[3],
    };
    
    if (collisionType === "cylinder" || collisionType === "capsule") {
      const radius = linkCollision.radius;
      const length = linkCollision.length;
      // Should probably use extraParams in the future
      // collisionShape.properties.extraParams = {length, radius};
      collisionShape.properties.scale = {
        x: radius * 2,
        y: radius * 2,
        z: length,
      };
    } else if (collisionType === "cube") {
      // Should probably use extraParams in the future
      // collisionShape.properties.extraParams = {x: sizeVec[0], y: sizeVec[1], z:sizeVec[2]};
      collisionShape.properties.scale = {
        x: linkCollision.x,
        y: linkCollision.y,
        z: linkCollision.z,
      };
    } else if (collisionType === "sphere") {
      const radius = linkCollision.radius;
      // collisionShape.properties.extraParams = {radius};
      collisionShape.properties.scale = {
        x: radius * 2,
        y: radius * 2,
        z: radius * 2,
      };
    }
    return collisionShape;
  }
};

const getParentLink = (parentJoint,joints,worldAttachment,prevLink) => {
  if (parentJoint === 'world') {
    return worldAttachment;
  } 
  const joint = joints.filter(j=>j.name === parentJoint)[0];
  console.log('found joint',joint)
  if (joint && joint.parentLink !== 'world') {
    return joint.parentLink
  } else if (!joint) {
    return prevLink
  }
  return worldAttachment
}
