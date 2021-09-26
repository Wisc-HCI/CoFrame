import React, { PureComponent } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer, Label } from 'recharts';

export function getPlotInfo({editorPane,setupTab,frame,primaryColor,focusData,secondaryFocusData,currentIssue}) {
    let tabs = [];
    tabs.push(
        {
            title:<span>Graph</span>,
            contents:<div style={{ width: '100%', height: '100%', margin: 'auto', fill: 'white'}}>
                {currentIssue.graphData.title !== '' && <h3 style={{textAlign: 'center'}}>{currentIssue.graphData.title}</h3>}
                <ResponsiveContainer>
                    <LineChart data={currentIssue.graphData.series}>
                        <CartesianGrid strokeDasharray="3 3"/>
                        <XAxis dataKey='x'>
                            <Label value={currentIssue.graphData.xAxisLabel} offset={0} position="insideBottom" />
                        </XAxis>
                        <YAxis label={{ value: currentIssue.graphData.yAxisLabel, angle: -90, position: 'insideLeft'}}/>
                        <Tooltip />
                        <Line type="monotone" dataKey="y" stroke={primaryColor} activeDot={{ r: 8 }} />
                    </LineChart>
                </ResponsiveContainer>
            </div>
        }
    );
    return tabs;
}