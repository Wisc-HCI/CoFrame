import React, { Component } from 'react'

import { ScrollablePane, ScrollbarVisibility } from 'office-ui-fabric-react/lib/ScrollablePane';

import { LoremIpsum } from 'react-lorem-ipsum';

export default class ParametersPanel extends Component {

    render() {

        const paddingWidth = 10; //px
        let width = this.props.width - 2 * paddingWidth;

        const paddingHeight = 10;
        let height = this.props.height - paddingHeight;

        let distanceFromTop = paddingHeight + this.props.distanceFromTop;

        console.log(`Distance from top: ${distanceFromTop}`);

        return (
            <div style={{ 
                    paddingLeft:`${paddingWidth}px`, 
                    paddingRight:`${paddingWidth}px`, 
                    width: `${width}px`, 
                    height: `${height}px`, 
                    maxHeight: `${height}px`
                }}
            >
              <div style={{display: "flex", flexDirection: "column", width: `${this.props.width}px`, position: "absolute", top: `${distanceFromTop}px`, bottom: "0px", backgroundColor: '#0f0'}}>
                    <div style={{flex: "1", backgroundColor: '#f00', position: "relative"}}>
                      <ScrollablePane styles={{root: {maxWidth: this.props.width}}}><LoremIpsum p={7} /></ScrollablePane>
                    </div>
              </div>
            </div>
        )
    }
}
