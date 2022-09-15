import shallow from "zustand/shallow";

export const stringEquality = (e1,e2) => JSON.stringify(e1) === JSON.stringify(e2)

export const arrayEqual = (a, b) => {
    if (a === b) return true;
    if (a == null || b == null) return false;
    if (a.length !== b.length) return false;

    // If you don't care about the order of the elements inside
    // the array, you should sort both arrays here.
    // Please note that calling sort on an array will modify that array.
    // you might want to clone your array first.

    for (var i = 0; i < a.length; ++i) {
        if (a[i] !== b[i]) return false;
    }
    return true;
}

export const objectEquality = (obj1, obj2) => {
    if (Array.isArray(obj1)){
        return arrayEqual(obj1, obj2);
    } else if (typeof(obj1) === typeof({}) && !Array.isArray(obj1)) {
        return shallow(obj1, obj2);
    } else if (typeof(obj1) === typeof(false)) {
        return obj1 === obj2;
    } else {
        return obj1 === obj2
    }
}

export const csArrayEquality = (arr1, arr2) => {
    let equal = true;

    if (arr1.length !== arr2.length) {
        return false;
    }

    for (let i = 0, n = arr1.length; i < n; i++) {
        equal = equal && objectEquality(arr1[i], arr2[i]);
    }
    return equal;
}