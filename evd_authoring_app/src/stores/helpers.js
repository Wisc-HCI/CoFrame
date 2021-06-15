import lodash from 'lodash';

export function flattenProgram(primitives,skills) {

    let flattenedPrimitives = [];
    let flattenedSkills = [];

    primitives.forEach(primitive=>{
        if (primitive.type.includes('hierarchical')) {
            let newPrimitive = lodash.omit(primitive,'primitives');
            newPrimitive.primitiveIds = primitive.primitives.map(primitive=>primitive.uuid);
            flattenedPrimitives.push(newPrimitive);
            const primitiveChildren = flattenProgram(primitive.primitives,[])[0];
            flattenedPrimitives = [...flattenedPrimitives, ...primitiveChildren];
        } else {
            flattenedPrimitives.push(primitive)
        }
    });
    skills.forEach(skill=>{
        if (skill.type.includes('hierarchical')) {
            let newSkill = lodash.omit(skill,'primitives');
            newSkill.primitiveIds = skill.primitives.map(primitive=>primitive.uuid);
            flattenedSkills.push(newSkill);
            const primitiveChildren = flattenProgram(skill.primitives,[])[0];
            flattenedPrimitives = [...flattenedPrimitives, ...primitiveChildren]
        }
    })

    return [flattenedPrimitives,flattenedSkills]
}