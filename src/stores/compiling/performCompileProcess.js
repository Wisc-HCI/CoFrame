import { ROOT_PATH, PREPROCESS_TYPES, POSTPROCESS_TYPES } from '../Constants';
import { handleUpdate } from './';
import { DATA_TYPES } from 'simple-vp';
import { createEnvironmentModel } from '../../helpers/geometry';
import init, {Solver} from 'coframe-rust';
// import { Solver } from '@people_and_robots/lively';

export const performCompileProcess = async (data) => {
    console.log('performCompilerProcess--inworker')
    const { programData, compiledData, objectTypes } = data;
    // Process the data without stalling the UI
    // const module = await loadLikModule();
    // console.warn('performing compile process')
    
    // const compModule = await loadCompilerModule();
    // console.warn(compModule.welcome("Harry"));
    // const poses = Object.values(programData).filter(d=>(d.type==='waypointType'||d.type==='locationType')&&d.dataType===DATA_TYPES.INSTANCE)
    // console.warn({input:poses,output:compModule.compile(poses)});

    let root = null;
    Object.values(programData).some(v=>{
        if (v.type==='programType') {
            root = v.id;
            return true
        } else {
            return false
        }
    })

    await init()

    // TODO: define scene based on the scene item instances
    const worldModel = createEnvironmentModel(programData)

    const compileModel = compiledData;

    // First, preprocess certain types:
    let memo = {};
    let compiledMemo = {};

    Object.values(programData)
        .filter(v=>PREPROCESS_TYPES.includes(v.type) && v.dataType === DATA_TYPES.INSTANCE)
        .forEach(data=>{
            // console.log('preprocessing',data)
            const computeProps = {
                data,
                objectTypes,
                context:programData,
                path:ROOT_PATH,
                memo,
                compiledMemo,
                module:{Solver},
                worldModel,
                compileModel
            }
            const {memo:newMemo, compiledMemo:newCompiledMemo} = handleUpdate(computeProps);
            // console.log({newMemo,compiledMemo})
            memo = {...memo,...newMemo};
            compiledMemo = {...compiledMemo, ...newCompiledMemo};
            // console.log('compiledMemo',{data,compiledMemo})
    })

    // This is recursive
    const computeProps = {
        data:programData[root],
        objectTypes,
        context:programData,
        path:ROOT_PATH,
        memo,
        compiledMemo,
        module:{Solver},
        worldModel,
        compileModel
    }
    const {memo:newMemo, compiledMemo:newCompiledMemo} = handleUpdate(computeProps);

    memo = {...memo,...newMemo};
    compiledMemo = {...compiledMemo, ...newCompiledMemo};

    // Snag any instances which were not directly accessible that may need to be processed.
    Object.values(programData)
        .filter(v=>POSTPROCESS_TYPES.includes(v.type) && v.dataType === DATA_TYPES.INSTANCE)
        .forEach(data=>{
            const computeProps = {
                data,
                objectTypes,
                context:programData,
                path:ROOT_PATH,
                memo,
                compiledMemo,
                module:{Solver},
                worldModel,
                compileModel
            }
            const {memo:newMemo, compiledMemo:newCompiledMemo} = handleUpdate(computeProps);
            memo = {...memo,...newMemo};
            compiledMemo = {...compiledMemo, ...newCompiledMemo};
    })
    console.log('performCompilerProcess--inworker ended')
    return {data: memo, compiledData: compiledMemo };
}