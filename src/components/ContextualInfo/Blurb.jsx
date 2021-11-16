import styled from 'styled-components';

export const Blurb = styled.div`
    display:block;
    background: rgba(100,100,100,0.2);
    padding: 15px 20px 15px 45px;
    border-radius: 4px;
    margin: 0 0 10px;
    position: relative;

    /*Font*/
    // font-family: Georgia, serif;
    font-size: 16px;
    line-height: 1.2;
    color: #999;
    // text-align: justify;

    /*Borders - (Optional)*/
    border-left: 15px solid ${props=>props.highlight} ;
    // border-right: 2px solid ${props=>props.highlight} ;
    
`;