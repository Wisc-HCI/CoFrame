import shallow from "zustand/shallow";

export const stringEquality = (e1,e2) => JSON.stringify(e1) === JSON.stringify(e2)

const arrayEqual = (a, b) => {
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

export const csArrayEquality = (arr1, arr2) => {
    let equal = true;

    if (arr1.length !== arr2.length) {
        return false;
    }

    for (let i = 0, n = arr1.length; i < n; i++) {
        // First index is always an object (lookup table)
        if (i === 0) {
            equal = equal && shallow(arr1[i], arr2[i]);
        } else if (Array.isArray(arr1[i])){
            equal = equal && arrayEqual(arr1[i], arr2[i]);
        } else if (typeof(arr1[i]) === typeof(false)) {
            equal = equal && (arr1[i] === arr2[i]);
        } else {
            equal = equal && (arr1[i] === arr2[i])
        }
    }
    return equal;
}