import React, { Component } from 'react';
import { Stack, Separator } from 'office-ui-fabric-react';

import { Tile } from './Tile';
import { Simulator } from './Simulator';
import { ProgramEditor } from './ProgramEditor';
import { FrameButtons } from './FrameButtons';
import { ExpertChecklist } from './ExpertChecklist';

export class Body extends Component {

    constructor(props) {
        super(props);

        this.state = {
            frame: 'safety',
        };

        this.onFrameButtonClicked = this.onFrameButtonClicked.bind(this);
    }

    onFrameButtonClicked(frame) {
        this.setState({ frame });
    }

    render() {
        const { frame } = this.state;
        const { layoutObj, theme, mainPadding } = this.props;

        console.log(layoutObj);

        return (
            <div
                style={{
                    width: `${layoutObj.body.width}px`,
                    height: `${layoutObj.body.height}px`,
                }}
            >
                <Stack horizontal>

                    <div style={{
                            paddingRight: `${mainPadding / 2}px`,
                            paddingTop: `${mainPadding}px`,
                            paddingBottom: `${mainPadding}px`,
                        }}
                    >
                        <Tile
                            theme={theme}
                            width={layoutObj.body.checklist.width - mainPadding / 2}
                            height={layoutObj.body.checklist.height - mainPadding}
                        >
                            <div
                                style={{
                                    width: layoutObj.body.checklist.header.width - 1.5 * mainPadding,
                                    height: layoutObj.body.checklist.header.height
                                }}
                            >
                                <div
                                    style={{
                                        fontSize: '25px',
                                        textAlign: 'center',
                                    }}
                                >
                                    <i>Checklist</i>
                                    <Separator />
                                </div>

                                <FrameButtons 
                                    frame={frame} 
                                    callback={this.onFrameButtonClicked} 
                                />
                            </div>
                            
                            <ExpertChecklist 
                                frame={frame}
                                width={layoutObj.body.checklist.body.width - mainPadding }
                                height={layoutObj.body.checklist.body.height - mainPadding}
                            />
                        </Tile>
                    </div>

                    <div
                        style={{
                            paddingLeft: `${mainPadding / 2}px`,
                            paddingRight: `${mainPadding / 2}px`,
                            paddingTop: `${mainPadding}px`,
                            paddingBottom: `${mainPadding}px`,
                        }}
                    >

                        <Tile
                            theme={theme}
                            width={layoutObj.body.simulator.width - mainPadding}
                            height={layoutObj.body.simulator.height - mainPadding}
                        >
                            <div
                                style={{
                                    fontSize: '25px',
                                    textAlign: 'center',
                                    width: layoutObj.body.simulator.header.width - 2 * mainPadding,
                                    height: layoutObj.body.simulator.header.height
                                }}
                            >
                                <i>Simulation</i>
                                <Separator />
                            </div>
                            <div 
                                style={{
                                    width: layoutObj.body.simulator.body.width - 2 * mainPadding,
                                    height: layoutObj.body.simulator.body.height,
                                }}
                            >
                                <Simulator frame={frame} />
                            </div>
                        </Tile>     
                    </div>

                    <div
                        style={{
                            paddingTop: `${mainPadding}px`,
                            paddingBottom: `${mainPadding}px`,
                            paddingLeft: `${mainPadding / 2}px`,
                        }}
                    >
                        <Tile 
                            theme={theme}
                            width={layoutObj.body.program.width - mainPadding / 2}
                            height={layoutObj.body.program.height - mainPadding}
                        >
                            <div
                                style={{
                                    height: `${layoutObj.body.program.header.height}px`,
                                    width: `${layoutObj.body.program.header.width - mainPadding / 2}px`,
                                    fontSize: '25px',
                                    textAlign: 'center'
                                }}
                            >
                                <i>Program</i>
                                <Separator />
                            </div>

                            <ProgramEditor 
                                width={layoutObj.body.program.body.width - mainPadding / 2}
                                height={layoutObj.body.program.body.height - mainPadding /2}
                            />
                        </Tile>  
                    </div>

                    
                </Stack>
            </div>
        );
    }
    
}