const path = require("path");
const HtmlWebpackPlugin = require("html-webpack-plugin");
const webpack = require("webpack");
const CracoSwcPlugin = require('craco-swc');

module.exports = {
    entry: path.resolve(__dirname, "./src/index.js"),
    target: 'web',
    webpack: {stats: {children: true}},
    devServer: {
        static: path.join(__dirname, "dist"),
        compress: true,
        port: 8080,
        hot: true
    },
    output: {
        publicPath: "auto",
    },
    resolve: {
        extensions: [".js", ".jsx"],
    },
    module: {
        rules: [
            {
                test: /\.css$/i,
                use: ["style-loader", "css-loader"],
            },
            {
                test: /\.svg$/,
                use: ['@svgr/webpack', 'url-loader'],
            },
            {
                test: /\.(js|jsx)$/,
                use: ["babel-loader"],
            },
            {
                test: /\.(glb|gltf)$/,
                use: {loader:'file-loader'}
            }
        ],
    },
    plugins: [
        // {plugin: CracoSwcPlugin},
        // new webpack.HotModuleReplacementPlugin(),
        {plugin: new HtmlWebpackPlugin({
            manifest: "./public/manifest.json",
            favicon: "./public/favicon.ico",
            template: "./public/index.html",
        })},
        {plugin: new webpack.ContextReplacementPlugin(/@people_and_robots/)}
    ],
    experiments: {
        asyncWebAssembly: true,
        syncWebAssembly: true,
        topLevelAwait: true
    }
};