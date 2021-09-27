import React from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer, Label } from 'recharts';

export function getPlotInfo({editorPane,setupTab,frame,primaryColor,focusData,secondaryFocusData,currentIssue}) {
    let tabs = [];
    const colors = currentIssue.graphData.lineColors;
    const showLegend = Object.keys(currentIssue.graphData.series[0]).length > 2;
    const intervalLength = currentIssue.graphData.series.length > 5 ? Math.floor(currentIssue.graphData.series.length / 5) : 0;
    tabs.push(
        {
            title:<span>Graph</span>,
            contents:<div style={{ width: '100%', minHeight:'200px', height: '100%', fill: 'white'}}>
                {currentIssue.graphData.title !== '' && <h3 style={{textAlign: 'center'}}>{currentIssue.graphData.title}</h3>}
                <ResponsiveContainer width="99%" aspect={3}>
                    <LineChart data={currentIssue.graphData.series}>
                        <CartesianGrid strokeDasharray="3 3"/>
                        <XAxis dataKey='x' interval={intervalLength}>
                            <Label value={currentIssue.graphData.xAxisLabel} offset={-2} position="insideBottom" />
                        </XAxis>
                        <YAxis label={{ value: currentIssue.graphData.yAxisLabel, angle: -90, position: 'insideLeft'}}/>
                        {Object.keys(currentIssue.graphData.series[0]).filter(entry=>entry!=='x').map((entry, idx) => {
                            return <Line type="linear" dataKey={entry} key={entry} stroke={colors[idx-1]} dot={false}/>
                        })}
                        <Tooltip contentStyle={{backgroundColor: '#141414'}} />
                        {showLegend && <Legend verticalAlign="top" layout="horizontal" align="right" />}
                    </LineChart>
                </ResponsiveContainer>
            </div>
        }
    );
    return tabs;
}