import styled from "styled-components";
import * as ScrollArea from "@radix-ui/react-scroll-area";

const StyledScrollArea = styled(ScrollArea.Root)`
  overflow: hidden;
  height: ${(props) =>
    props.$containerHeight ? `${props.$containerHeight}px` : "100%"};
  width: ${(props) =>
    props.$containerWidth ? `${props.$containerWidth}px` : "100%"};
`;

const StyledViewport = styled(ScrollArea.Viewport)`
  width: 100%;
  height: 100%;
  border-radius: inherit;
  padding: 4px;
`;

const StyledScrollbar = styled(ScrollArea.Scrollbar)`
  display: flex;
  user-select: none;
  touch-action: none;
  padding: 2px;
  background: #55555525;
  transition: background 160ms ease-out;
  &:hover: {
    background: #45454540;
  }
`;

const VerticalScrollBar = styled(StyledScrollbar)`
  width: 8px;
`;

const HorizontalScrollBar = styled(StyledScrollbar)`
  height: 8px;
  flex-direction: column;
`;

const StyledScrollThumb = styled(ScrollArea.Thumb)`
  flex: 1;
  background: #eeeeee66;
  border-radius: 8px;
`;

export const ScrollRegion = ({
    children,
    horizontal=false,
    vertical=true,
    height='100%',
    width='100%',
  }) => (
    <StyledScrollArea $containerHeight={height} $containerWidth={width}>
      <StyledViewport>{children}</StyledViewport>
      {horizontal && (
        <HorizontalScrollBar orientation="horizontal">
          <StyledScrollThumb />
        </HorizontalScrollBar>
      )}
      {vertical && (
        <VerticalScrollBar orientation="vertical">
          <StyledScrollThumb />
        </VerticalScrollBar>
      )}
      <ScrollArea.Corner />
    </StyledScrollArea>
  );