import React, { Component } from 'react'

import { Button, Divider } from '@fluentui/react-northstar';
import { PlayIcon, RetryIcon, PauseThickIcon } from '@fluentui/react-icons-northstar';

class Footer extends Component {
    constructor(props) {
        super(props);

        this.state = {
            resetEnabled: true,
            playEnabled: true,
            pauseEnabled: false
        };

        this.onReset.bind(this);
        this.onPlay.bind(this);
        this.onPause.bind(this);
    }

    render() {

        let Play = null;
        if (this.props.type === "PLAY_CONTROLS") {
            return (
                <footer style={FooterStyle}>
                    <Divider styles={{paddingBottom: "5px"}} size={2} color='brand'/>

                    <Button 
                        circular={true} 
                        icon={<RetryIcon />} 
                        iconOnly 
                        title="Reset" 
                        disabled={!this.state.resetEnabled} 
                        onClick={this.onReset}
                    />
                    <Button 
                        circular={true} 
                        icon={<PlayIcon />} 
                        iconOnly 
                        title="Play" 
                        disabled={!this.state.playEnabled} 
                        onClick={this.onPlay}
                    />
                    <Button 
                        circular={true} 
                        icon={<PauseThickIcon />} 
                        iconOnly 
                        title="Pause" 
                        disabled={!this.state.pauseEnabled} 
                        onClick={this.onPause}
                    />
                </footer>
            )
        } else if (this.props.type === "BLANK") {
            return (
                <footer style={FooterStyle}>
                    <Divider size={2} color='brand'/>
                </footer>
            );
        }  
    }

    onReset(e) {
        console.log(e);
    }

    onPlay(e) {
        console.log(e);
    }

    onPause(e) {
        console.log(e);
    }
}

const FooterStyle = {
   position: 'absolute',
   bottom: '0',
   width: '100%',
   textAlign: 'center',
   background: '#11100F'
}



export default  Footer;