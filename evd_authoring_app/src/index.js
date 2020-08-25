import React from 'react';
import ReactDOM from 'react-dom';
import App from './App';
import * as serviceWorker from './serviceWorker';

import {
  loadTheme,
  styled,
  classNamesFunction,
  initializeIcons
} from "office-ui-fabric-react";

import { styles } from "./App.styles"; // We'll have to create the App.styles.js file
import "./styles.css";

// Importing our theme definitions
import { dark } from "./themes/dark";
import { light } from "./themes/light";

const getClassNames = classNamesFunction();

// Loads the light theme by default
let currentTheme = dark;
loadTheme(currentTheme);

// Toggles between the light & dark theme
const toggleTheme = () => {
  currentTheme = currentTheme === light ? dark : light;
  loadTheme(currentTheme);
};

// Initialize icons in case this example uses them
initializeIcons();

function AppThemeWrapper(props) {
  const {styles, theme } = props;

  const classNames = getClassNames(styles, theme);

  return (
    <div className={classNames.root}>
      <App theme={theme}/>
    </div>);
}

// Passes the theme and styles as props to our component
const StyledApp = styled(AppThemeWrapper, styles);

ReactDOM.render(
  <StyledApp />,
  document.getElementById('root')
);

// If you want your app to work offline and load faster, you can change
// unregister() to register() below. Note this comes with some pitfalls.
// Learn more about service workers: https://bit.ly/CRA-PWA
serviceWorker.unregister();
