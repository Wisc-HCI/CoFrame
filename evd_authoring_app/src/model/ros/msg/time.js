
export const TimeNowUnwrapped = () => {
    const currentTime = new Date();
    const secs = Math.floor(currentTime.getTime() / 1000);
    const nsecs = Math.round(1000000000*(currentTime.getTime()/1000-secs));
    return TimeUnwrapped(secs, nsecs);
};

export const TimeUnwrapped = (secs, nsecs) => {
    return {secs, nsecs};
};
