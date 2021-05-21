import React from 'react';
import ReactDOM from 'react-dom';
import { 
    BrowserRouter as Router, 
    Route 
} from 'react-router-dom';

import { initializeIcons } from '@fluentui/react/lib/Icons';
import { loadTheme, styled, classNamesFunction } from '@fluentui/react';

// App styling
import styles from './styles';
import './index.css';
import dark from './themes/dark';
import light from './themes/light';
import * as frameStyles from './frameStyles';

// Application
import { App } from './App';
import * as serviceWorker from './serviceWorker';

// Model
import { evdScriptBlocklyInitialize } from './model/evdScript';


evdScriptBlocklyInitialize();
initializeIcons();


const getClassNames = classNamesFunction();


// Loads the light theme by default
let currentThemeName = 'dark';
let currentTheme = dark;
loadTheme(currentTheme);


// Toggles between the light & dark theme
const toggleTheme = () => {
    currentThemeName = currentThemeName === 'light' ? 'dark' : 'light';

    switch (currentThemeName) {
        case 'light':
            currentTheme = light;
            break;
        case 'dark':
            currentTheme = dark;
            break;
        default:
            currentTheme = dark;
            break;
    }

    loadTheme(currentTheme);
};


function AppThemeWrapper(props) {

    const { styles, theme } = props;
    const classNames = getClassNames(styles, theme);

    return (
        <div className={classNames.root}>
            <Router>
                <Route exact path="/" render={() => (
                    <App
                        theme={theme}
                        frameStyles={frameStyles.default}
                        toggleTheme={toggleTheme}
                        themeName={currentThemeName}
                        useChecklist={true}
                    />
                )}/>
            </Router>
        </div>
    );
}

// Passes the theme and styles as props to our component
const StyledApp = styled(AppThemeWrapper, styles);

ReactDOM.render(<StyledApp />, document.getElementById('root'));

// If you want your app to work offline and load faster, you can change
// unregister() to register() below. Note this comes with some pitfalls.
// Learn more about service workers: https://bit.ly/CRA-PWA
serviceWorker.unregister();
