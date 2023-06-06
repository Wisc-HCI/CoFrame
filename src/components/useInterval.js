import { useState } from 'react';

export const useInterval = (length) => {

    const [runtimeProps, setRuntimeProps] = useState({});
    const [handle, setHandle] = useState(null);

    const startIntervalFn = ({
      props = {},
      intervalFn = (props) => (null)
    }) => {
      const interval = setInterval(() => {
        intervalFn({...props,...runtimeProps});
      }, length);
      setHandle(interval)
    }

    const clearIntervalFn = () => {
      clearInterval(handle)
    }

    return {startIntervalFn, setRuntimeProps, clearIntervalFn}
}