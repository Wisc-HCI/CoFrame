import { v4 as uuidv4 } from 'uuid';

const generateUuid = (type) => {
    return `${type}-${uuidv4()}`;
};

export { generateUuid };