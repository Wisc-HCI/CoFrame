import { ROOT_PATH, PREPROCESS_TYPES, POSTPROCESS_TYPES } from '../Constants';
import { handleUpdate } from './';
import { DATA_TYPES } from 'open-vp';
import { createEnvironmentModel } from '../../helpers/geometry';
import * as coframerust from 'coframe-rust';
// import { Solver } from '@people_and_robots/lively';

export const performCompileProcess = async (data) => {
    console.log('performCompilerProcess -- in worker')
    
    const { programData, compiledData, objectTypes, module:defaultmodule } = data;
    if (!defaultmodule) {
        await coframerust.default();
    }
    // Process the data without stalling the UI
    // const module = await loadLikModule();
    // console.warn('performing compile process')

    let root = null;
    Object.values(programData).some(v=>{
        if (v.type==='programType') {
            root = v.id;
            return true
        } else {
            return false
        }
    })

    const module = defaultmodule || coframerust;

    // TODO: define scene based on the scene item instances
    const worldModel = createEnvironmentModel(programData)

    const compileModel = compiledData;

    // First, preprocess certain types:
    let memo = {};
    let compiledMemo = {};

    Object.values(programData)
        .filter(v=>PREPROCESS_TYPES.includes(v.type) && v.dataType === DATA_TYPES.INSTANCE)
        .forEach(data=>{
            // console.log('preprocessing',data.type)
            const computeProps = {
                data,
                objectTypes,
                context:programData,
                path:ROOT_PATH,
                memo,
                compiledMemo,
                module,
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
        module,
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
                module,
                worldModel,
                compileModel
            }
            const {memo:newMemo, compiledMemo:newCompiledMemo} = handleUpdate(computeProps);
            memo = {...memo,...newMemo};
            compiledMemo = {...compiledMemo, ...newCompiledMemo};
    })
    console.log('performCompilerProcess -- in worker [complete]')
    return {data: memo, compiledData: compiledMemo };
}