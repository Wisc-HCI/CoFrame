import { useState, useEffect } from 'react';
import useStore from '../stores/Store';

export const useTime = (totalLength) => {

    const clock = useStore((state) => state.clock);
    const [time, setTime] = useState(clock.getElapsed());

    useEffect(() => {
        const interval = setInterval(() => {
          const time = (clock.getElapsed() * 1000) % totalLength;
          setTime(time);
        }, 25);
        return () => clearInterval(interval);
      });

    return time
}