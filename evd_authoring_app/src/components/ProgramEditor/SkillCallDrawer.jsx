import React from 'react';
import { acceptLookup } from './acceptLookup';
import useStore from '../../stores/Store';
import { generateUuid } from '../../stores/generateUuid';
import { SkillCallBlock } from './SkillCallBlock';

const skill2Call = (skill) => {
    let parameters = {skill_uuid:skill.uuid};
    skill.arguments.forEach(arg=>{parameters[arg.uuid] = null})

    return {
        type: 'node.primitive.skill-call.',
        uuid: generateUuid('skill-call'),
        name: `Execute Skill: ${skill.name}`,
        editable: true,
        deleteable: true,
        description: '',
        parameters
    }
}

export const SkillCallDrawer = (_) => {

    const ancestors = [
        { uuid: 'drawer', ...acceptLookup.drawer.default }
    ];

    const skills = useStore(state => Object.values(state.data.skills));

    return (
        <React.Fragment>
            {skills.map(skill=>(
                <div key={skill.uuid} style={{paddingTop: 5}}>
                    <SkillCallBlock
                        staticData={skill2Call(skill)}
                        ancestors={ancestors}
                        idx={0}
                        parentData={{ type: 'drawer', uuid: 'drawer' }}
                        dragBehavior='copy'
                        context={{}}
                    />
                </div>
            ))}
        </React.Fragment>
    );
};
