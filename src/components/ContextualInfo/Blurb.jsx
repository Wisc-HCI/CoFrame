import styled from "@emotion/styled";

export const Blurb = styled.div({
    display:'block',
    background: 'rgba(100,100,100,0.2)',
    padding: '3px 10px 15px 15px',
    borderRadius: '4px',
    margin: '0 0 10px',
    position: 'relative',

    /*Font*/
    // font-family: Georgia, serif;
    fontSize: '16px',
    lineHeight: 1.2,
    color: '#999'
},
(props) => {{borderLeft: props.highlight}}
);