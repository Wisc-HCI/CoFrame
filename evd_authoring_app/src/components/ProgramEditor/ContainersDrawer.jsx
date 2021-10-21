import React from 'react';
import { TrajectoryBlock } from './TrajectoryBlock';
import { CanvasBlock } from './CanvasBlock';
import { ActionBlock } from './ActionBlock';
import { fromContainerTemplate } from '../../stores/templates';
import { acceptLookup } from './acceptLookup';

export const ContainersDrawer = (_) => {

    const ancestors = [
        { uuid: 'drawer', ...acceptLookup.drawer.default }
    ];

    return (
        <React.Fragment>
            <div key='trajectory' style={{ paddingTop: 5 }} >
                <TrajectoryBlock
                    key='trajectory'
                    staticData={fromContainerTemplate('trajectory')}
                    ancestors={ancestors}
                    idx={0}
                    parentData={{ type: 'drawer', uuid: 'drawer' }}
                    dragBehavior='copy'
                    context={{}} />
            </div>
            <div key='skill' style={{ paddingTop: 5 }} >
                <CanvasBlock
                    key='skill'
                    staticData={fromContainerTemplate('skill')}
                    ancestors={ancestors}
                    idx={1}
                    parentData={{type: 'drawer',uuid: 'drawer'}}
                    dragBehavior='copy'
                    context={{ }}/>
            </div>
            <div key='hierarchical' style={{paddingTop: 5}} >
                <ActionBlock
                    key='hierarchical'
                    staticData={fromContainerTemplate('hierarchical')}
                    ancestors={ancestors}
                    idx={2}
                    parentData={{type: 'drawer',uuid: 'drawer'}}
                    dragBehavior='copy'
                    context={{ }}/>
            </div>
        </React.Fragment>
    );
};
