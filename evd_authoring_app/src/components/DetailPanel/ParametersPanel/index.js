import React, { Component } from 'react'

import { ScrollablePane, ScrollbarVisibility } from 'office-ui-fabric-react/lib/ScrollablePane';
import { Sticky, StickyPositionType } from 'office-ui-fabric-react/lib/Sticky';

export default class ParametersPanel extends Component {
    render() {

        const colors = ['#eaeaea', '#dadada', '#d0d0d0', '#c8c8c8', '#a6a6a6', '#c7e0f4', '#71afe5', '#eff6fc', '#deecf9'];

        const items = Array.from({ length: 5 }).map((item, index) => ({
            color: colors.splice(Math.floor(Math.random() * colors.length), 1)[0],
            text: "test",
            index,
          }));

        const createContentArea = (item) => (
            <div
              key={item.index}
              style={{
                backgroundColor: item.color,
              }}
            >
              <Sticky stickyPosition={StickyPositionType.Both}>
                <div>Sticky Component #{item.index + 1}</div>
              </Sticky>
              <div>{item.text}</div>
            </div>
          );

        const contentAreas = items.map(createContentArea);

        const paddingWidth = 10; //px

        return (
            <div style={{ paddingLeft:`${paddingWidth}px`, paddingRight:`${paddingWidth}px`, width: `${this.props.width}px`}}>
                <p>Test</p>
            </div>
        )
    }
}
