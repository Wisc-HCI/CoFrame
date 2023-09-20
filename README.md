# Building and Running CoFrame
## Dependencies
[NodeJS](https://nodejs.org/en)
[Yarn](https://classic.yarnpkg.com/lang/en/docs/install/#windows-stable)
[Rust](https://www.rust-lang.org/tools/install)
[wasm-pack](https://rustwasm.github.io/wasm-pack/)

## Steps
1. Run `yarn install` to install all packages
2. Run `yarn build:wasm` to build the rust package
3. Run `yarn start` to launch the Tauri client or run `yarn start:web` to begin the webserver.

# Build Docker Image
1. Update the base path in `vite.config.js` to be `/` instead of `/CoFrame/`
2. Install dependencies using `yarn install` 
3. Build the rust package using `yarn build:wasm`
3. Build the website using `yarn build`
4. Build the docker image and run it using `docker compose up`
