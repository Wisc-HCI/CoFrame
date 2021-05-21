import create from "zustand";
import produce from "immer";

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);



const store = (set) => ({
    url: 'ws://localhost:9090',
    // SetURL resets ROS
    setUrl: (url) => set({url:url,connected:false,ros:null}),
    ros: null,
    connected: false,
    onConnection: ()=>{},
    onError: ()=>{},
    onClose: ()=>{},
    connect: async () => {
        const ros = new ROSLIB.Ros();
        ros.on('connection', state.onConnection);
        ros.on('error', state.onError);
        ros.on('close',state.onClose);
        await ros.connect({url: state.url})
        set({ros: ros})
    }
});

const useGuiStore = create(immer(store));

const onConnection = () => {
    useGuiStore.setState({connected:true});
    window.alert('ROS is now connected');
}

const onError = (error) => {
    useGuiStore.setState({connected:false});
    window.alert('ROS encountered an error!');
    console.log('ros connection encountered an error', error);
}

const onClose = () => {
    useGuiStore.setState({connected:false});
    window.alert('ROS connection is closed');
}

useGuiStore.setState({onConnection:onConnection,onError:onError,onClose:onClose})

export default useGuiStore;