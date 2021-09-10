import { ExecuteMacrosBlock } from '../ExecuteMacroBlock';
import {PrimitiveBlock} from '../PrimitiveBlock';
import {SkillBlock} from '../SkillBlock';
import { TrajectoryBlock } from '../TrajectoryBlock';
import {UUIDBlock} from '../UUIDBlock';

export const childLookup = {
    'primitive': PrimitiveBlock,
    'skill': SkillBlock,
    'executeskill': ExecuteMacrosBlock,
    'uuid': UUIDBlock,
    'trajectory': TrajectoryBlock
}