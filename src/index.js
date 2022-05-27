import React from 'react';
import ReactDOM, {render} from 'react-dom';
import App from './App';
import './index.css' 
import * as serviceWorker from './serviceWorker';

import { createRoot } from 'react-dom/client';
const container = document.getElementById('root');

// temporarily use old react 17-style rendering
const root = createRoot(container);
root.render(<App/>);
// render(<App/>,container)

// If you want your app to work offline and load faster, you can change
// unregister() to register() below. Note this comes with some pitfalls.
// Learn more about service workers: https://bit.ly/CRA-PWA
serviceWorker.register();
