import React from 'react';
import { acceptLookup } from '../acceptLookup';
import useStore from '../../../stores/Store';
import { generateUuid } from '../../../stores/generateUuid';
import {SortableContext} from '@dnd-kit/sortable';
import { Block } from '../Blocks';

const skill2Call = (skill) => {
    let parameters = {skill_uuid:skill.uuid};
    skill.arguments.forEach(arg=>{parameters[arg.uuid] = null})

    return {
        type: 'skill-call',
        uuid: generateUuid('skill-call'),
        name: `Execute Skill: ${skill.name}`,
        readonly: false,
        description: '',
        parameters
    }
}

export const SkillCallDrawer = (_) => {

    const ancestors = [
        { uuid: 'drawer', ...acceptLookup.drawer.default }
    ];

    const skills = useStore(state => Object.values(state.data).filter(v=>v.type==='skill'));

    return (
        <SortableContext items={skills.map(item=>item.uuid)} id={'skill-call-drawer'}>
            {skills.map(item=>(
                <Block ancestors={ancestors} staticData={skill2Call(item)} context={{}} dragDisabled={false}/>
            ))}
        </SortableContext>
    );
};
