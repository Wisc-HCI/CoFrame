import React from 'react';
import ReactDOM from 'react-dom';

import { initializeIcons } from 'office-ui-fabric-react/lib/Icons';
import { loadTheme, styled, classNamesFunction } from 'office-ui-fabric-react';

// App styling
import styles from './App.styles';
import './styles.css';
import dark from './themes/dark';
import light from './themes/light';

// Application
import App from './App';
import * as serviceWorker from './serviceWorker';

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
      currentThemeName = 'dark';
      currentTheme = dark;
      break;
  }

  loadTheme(currentTheme);
};

function AppThemeWrapper(props) {
  // eslint-disable-next-line no-shadow
  const { styles, theme } = props;

  const classNames = getClassNames(styles, theme);

  return (
    <div className={classNames.root}>
      <App
        theme={theme}
        toggleTheme={toggleTheme}
        themeName={currentThemeName}
      />
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